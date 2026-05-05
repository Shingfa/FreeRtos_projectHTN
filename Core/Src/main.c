/* USER CODE BEGIN Header */
/**
  * ============================================================================
  * ĐỒ ÁN NHÚNG CƠ BẢN VỚI FREERTOS
  * ============================================================================
  * SƠ ĐỒ CHÂN (PINOUT):
  * - I2C1 (BME280 & LCD): PB8 (SCL), PB9 (SDA) - Chân Remap
  * - MQ2 (Khí Gas)      : PA4 (ADC1_IN4)
  * - PIR (Chuyển động)  : PA0 (EXTI0 - Ngắt cạnh lên)
  * - Relay (Quạt)       : PA3 (Active HIGH / LOW tùy module)
  * - LED Xanh (An toàn) : PA11
  * - LED Đỏ + Còi       : PB15 (Báo động)
  * - LED System         : PC13 (Nháy báo hiệu MCU đang sống)
  *
  * CÁC NÚT NHẤN (Kéo xuống GND, dùng Pull-up nội):
  * - Nút CHỌN / MENU    : PA15 (Đã giải phóng JTAG để dùng)
  * - Nút TĂNG (+)       : PB14
  * - Nút GIẢM (-)       : PA10
  * - Nút QUẠT (On/Off)  : PB12
  * ============================================================================
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <stdbool.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include <stdio.h>
#include "CLCD_I2C.h"
#include "bme280.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float nhietDo;
    float doAm;
    uint32_t khiGas;
} DuLieuHeThong_t;

typedef enum {
    NUT_TANG,
    NUT_GIAM,
    NUT_CHON,
    NUT_QUAT,   // Bật/tắt quạt thủ công
    KHONG_NHAN
} TrangThaiNutNhan_t;
/* USER CODE END PTD */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
BME280_HandleTypedef bme280;
CLCD_I2C_Name lcd1;

QueueHandle_t QueueCamBien;
QueueHandle_t QueueNutNhan;
SemaphoreHandle_t MutexI2C;     // Bảo vệ đường truyền I2C (tránh đụng độ giữa BME280 và LCD)
SemaphoreHandle_t SemaPIR;      // Kích hoạt Task An Ninh từ ngắt EXTI0
TimerHandle_t TimerBaoDong;     // Hẹn giờ tắt còi báo động PIR sau 5s

int nguongKhiGas = 1200;        // Ngưỡng khí Gas mặc định
uint8_t cheDoCaiDat = 0;        // Cờ chuyển đổi giữa Màn hình chính và Màn hình Cài đặt
volatile uint8_t pir_alarm = 0; // Biến toàn cục theo dõi trạng thái từ PIR
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void Task_CamBien(void *p);
void Task_HienThi_DieuKhien(void *p);
void Task_NutNhan(void *p);
void Task_AnNinh(void *p);
void Callback_TimerBaoDong(TimerHandle_t xTimer);
/* USER CODE END PFP */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  // Mở khóa JTAG để lấy lại chân PA15 làm nút bấm
  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  // Bật LED hệ thống để test
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  // Chỉ khởi tạo LCD ở đây (không khởi tạo BME280 ở main để tránh treo I2C)
  CLCD_I2C_Init(&lcd1, &hi2c1, 0x4E, 16, 2);

  // Khởi tạo các đối tượng FreeRTOS
  QueueCamBien = xQueueCreate(5, sizeof(DuLieuHeThong_t));
  QueueNutNhan = xQueueCreate(10, sizeof(TrangThaiNutNhan_t));
  MutexI2C = xSemaphoreCreateMutex();
  SemaPIR = xSemaphoreCreateBinary();
  TimerBaoDong = xTimerCreate("BaoDong", pdMS_TO_TICKS(5000), pdFALSE, 0, Callback_TimerBaoDong);

  // Tạo các Task (Ưu tiên: An ninh > Cảm Biến/Hiển Thị/Nút Nhấn)
  xTaskCreate(Task_CamBien,           "CamBien", 256, NULL, 2, NULL);
  xTaskCreate(Task_HienThi_DieuKhien, "HienThi", 256, NULL, 2, NULL);
  xTaskCreate(Task_NutNhan,           "NutNhan", 128, NULL, 2, NULL);
  xTaskCreate(Task_AnNinh,            "AnNinh",  128, NULL, 3, NULL);
  /* USER CODE END 2 */

  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osKernelStart();
  while (1) {}
}

/* USER CODE BEGIN 4 */

/**
  * @brief Ngắt ngoài phục vụ duy nhất cho cảm biến PIR (PA0)
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == GPIO_PIN_0) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // Báo hiệu cho Task_AnNinh biết có trộm
        xSemaphoreGiveFromISR(SemaPIR, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
  * @brief Task quét nút nhấn (Dùng phần mềm để chống dội phím thay cho ngắt)
  */
void Task_NutNhan(void *p) {
    TrangThaiNutNhan_t msg;
    for(;;) {
        msg = KHONG_NHAN;

        // --- Nút CHỌN / MENU (PA15) ---
        if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET) {
            vTaskDelay(pdMS_TO_TICKS(20)); // Delay 20ms để lọc nhiễu cơ học (chống dội)
            if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET) {
                msg = NUT_CHON;
                xQueueSend(QueueNutNhan, &msg, 0);
                // Vòng lặp chờ người dùng nhả nút (đảm bảo bấm 1 lần chỉ tính 1 lần)
                while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET) { vTaskDelay(10); }
            }
        }
        // --- Nút TĂNG (PB14) ---
        else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET) {
            vTaskDelay(pdMS_TO_TICKS(20));
            if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET) {
                msg = NUT_TANG;
                xQueueSend(QueueNutNhan, &msg, 0);
                while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET) { vTaskDelay(10); }
            }
        }
        // --- Nút GIẢM (PA10) ---
        else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET) {
            vTaskDelay(pdMS_TO_TICKS(20));
            if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET) {
                msg = NUT_GIAM;
                xQueueSend(QueueNutNhan, &msg, 0);
                while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET) { vTaskDelay(10); }
            }
        }
        // --- Nút QUẠT THỦ CÔNG (PB12) ---
        else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) {
            vTaskDelay(pdMS_TO_TICKS(20));
            if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) {
                msg = NUT_QUAT;
                xQueueSend(QueueNutNhan, &msg, 0);
                while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) { vTaskDelay(10); }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(30)); // Tần số quét phím
    }
}

/**
  * @brief Task thu thập dữ liệu cảm biến định kỳ
  */
void Task_CamBien(void *p) {
    DuLieuHeThong_t duLieu = {0, 0, 0};

    // Khởi tạo BME280 bên trong Task để tận dụng hàm HAL_Delay một cách an toàn
    bme280.hi2c = &hi2c1;
    bme280.dev_addr = 0xEC; // Địa chỉ I2C của BME280 (Gắn SDO xuống GND)

    if(xSemaphoreTake(MutexI2C, portMAX_DELAY) == pdTRUE) {
        BME280_Init(&bme280);
        xSemaphoreGive(MutexI2C);
    }

    for(;;) {
        // 1. Đọc MQ2 qua bộ chuyển đổi ADC
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
            duLieu.khiGas = HAL_ADC_GetValue(&hadc1);
        }

        // 2. Đọc Nhiệt độ & Độ ẩm qua I2C (Có Mutex bảo vệ tránh đụng màn hình LCD)
        if(xSemaphoreTake(MutexI2C, pdMS_TO_TICKS(100)) == pdTRUE) {
            BME280_ReadTemperature(&bme280, &duLieu.nhietDo);
            BME280_ReadHumidity(&bme280, &duLieu.doAm);
            xSemaphoreGive(MutexI2C);
        }

        // 3. Gửi dữ liệu vào Queue cho Task Hiển thị
        xQueueSend(QueueCamBien, &duLieu, 0);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Cập nhật mỗi 1 giây
    }
}

/**
  * @brief Task não bộ: Xử lý logic, xuất tín hiệu Relay/LED và hiển thị LCD
  */
void Task_HienThi_DieuKhien(void *p) {
    DuLieuHeThong_t duLieu = {0, 0, 0};
    TrangThaiNutNhan_t msgNutNhan;
    char dong1[17], dong2[17];
    bool update_lcd = true;
    bool quat_thu_cong = false; // Cờ nhớ trạng thái bật quạt thủ công

    for(;;) {
        update_lcd = false;

        // ====================================================================
        // 1. XỬ LÝ SỰ KIỆN NÚT NHẤN
        // ====================================================================
        while(xQueueReceive(QueueNutNhan, &msgNutNhan, 0) == pdPASS) {
            if(msgNutNhan == NUT_CHON) cheDoCaiDat = !cheDoCaiDat;

            if(cheDoCaiDat && msgNutNhan == NUT_TANG) nguongKhiGas += 100;
            if(cheDoCaiDat && msgNutNhan == NUT_GIAM) nguongKhiGas -= 100;
            if(nguongKhiGas < 0) nguongKhiGas = 0;

            // Bật/tắt cờ quạt thủ công
            if(msgNutNhan == NUT_QUAT) {
                quat_thu_cong = !quat_thu_cong;
            }
            update_lcd = true; // Bấm nút là reset LCD
        }
        // ====================================================================
                // 2 & 3. NHẬN DỮ LIỆU CẢM BIẾN VÀ ĐIỀU KHIỂN ĐỒNG BỘ
                // ====================================================================
                if(xQueueReceive(QueueCamBien, &duLieu, pdMS_TO_TICKS(50)) == pdPASS) {

                    // Gom điều kiện bật quạt vào một biến
                    bool dieu_kien_quat = (duLieu.khiGas > nguongKhiGas || duLieu.nhietDo > 40.0f || quat_thu_cong);

                    // Đèn đỏ bật khi: Quạt chạy HOẶC có cảnh báo PIR
                    if(dieu_kien_quat || pir_alarm) {
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
                        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
                    } else {
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
                        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
                    }

                    // Điều khiển Rơ-le quạt (Low-level trigger)
                    if(dieu_kien_quat) {
                        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); // BẬT QUẠT
                    } else {
                        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);   // TẮT QUẠT
                    }
                    update_lcd = true;
                }
        // 4. HIỂN THỊ LÊN MÀN HÌNH LCD
        // ====================================================================
        if(update_lcd) {
            if(xSemaphoreTake(MutexI2C, portMAX_DELAY) == pdTRUE) {
                if(!cheDoCaiDat) {
                    sprintf(dong1, "Temp:%dC  Hum:%d%% ", (int)duLieu.nhietDo, (int)duLieu.doAm);
                    sprintf(dong2, "Gas:%-4lu  PIR:%-2s", duLieu.khiGas, pir_alarm ? "!!" : "OK");
                } else {
                    sprintf(dong1, "CAI DAT KHI GAS ");
                    sprintf(dong2, "Nguong: %-4d    ", nguongKhiGas);
                }

                CLCD_I2C_SetCursor(&lcd1, 0, 0);
                CLCD_I2C_WriteString(&lcd1, dong1);
                CLCD_I2C_SetCursor(&lcd1, 0, 1);
                CLCD_I2C_WriteString(&lcd1, dong2);

                xSemaphoreGive(MutexI2C);
            }
        }
    }
}

/**
  * @brief Task ưu tiên cao: Xử lý kích hoạt báo động khi PIR phát hiện trộm
  */
void Task_AnNinh(void *p) {
    for(;;) {
        // Task sẽ block (đứng yên không tốn CPU) cho đến khi Ngắt EXTI0 cho phép (Give Semaphore)
        if(xSemaphoreTake(SemaPIR, portMAX_DELAY) == pdTRUE) {
            pir_alarm = 1; // Bật cờ báo động để ngắt cờ xanh, bật cờ đỏ

            // Khởi động bộ đếm 5 giây. Hết 5 giây còi sẽ tự động tắt
            xTimerStart(TimerBaoDong, 0);
        }
    }
}

/**
  * @brief Hàm Callback gọi khi TimerBaoDong đếm xong 5 giây
  */
void Callback_TimerBaoDong(TimerHandle_t xTimer) {
    pir_alarm = 0; // Hủy cờ báo động
}
/* USER CODE END 4 */

/* CÁC HÀM CẤU HÌNH HỆ THỐNG BÊN DƯỚI ĐƯỢC GIỮ NGUYÊN (SYSTEM CLOCK, GPIO, I2C, ADC) */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) { Error_Handler(); }
}

static void MX_ADC1_Init(void) {
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}

static void MX_I2C1_Init(void) {
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  // ĐẶT TRẠNG THÁI BAN ĐẦU: PA3 LÀ SET (TẮT QUẠT), PA11 LÀ RESET (LED XANH SÁNG)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15|GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  // CẤU HÌNH PA3: OPEN-DRAIN (OD) ĐỂ ĐIỀU KHIỂN RELAY 5V
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // CẤU HÌNH PA11: PUSH-PULL (PP) CHO LED XANH
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void StartDefaultTask(void const * argument) {
  for(;;) { osDelay(1); }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM4) { HAL_IncTick(); }
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {}
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif /* USE_FULL_ASSERT */
