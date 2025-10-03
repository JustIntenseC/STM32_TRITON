
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

volatile float x = 0, y = 0, z = 0;

volatile uint32_t periods[3] = {0};        // Tx, Ty, Tz
volatile uint32_t last_capture[3] = {0};
volatile uint8_t new_data_ready = 0;     


const float Ax_default = -468078.0f;
const float Ay_default = -415903.0f;
const float Az_default = -287031.0f;

const float Bx_default = -588544.0f;
const float By_default = -704679.0f;
const float Bz_default = -326929.0f;

const float Cx_default = 3027770.9f;
const float Cy_default = 3246825.6f;
const float Cz_default = 3292740.4f;

const float Dx_default = 14.4919f;
const float Dy_default = 11.3251f;
const float Dz_default = 11.53f;

float Mxx = 1, Mxy = 0, Mxz = 0; 
float Myx = 0, Myy = 1, Myz = 0;
float tc = 0.0000;
float tfactor = 1.0f;
const float t0 = 25.00;
SD_HandleTypeDef hsd;
HAL_SD_CardStatusTypeDef statusSD;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim2_2;
TIM_HandleTypeDef htim2_3;

char TxBuffer[250];
uint32_t primask = 0;
volatile uint8_t file_is_open = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SDIO_SD_Init(void);
 void SDIO_SDCard_Read(void);
static FRESULT open_log_file(void);
void DMA_Write(void);
static void MX_TIM2_Init(void);
float period_to_field(const uint32_t period, float A, float B, float C, float D);
static void Enable_Magnetometer(void);


static void USB_CDC_Print(char* TxStr)
{
    CDC_Transmit_FS((uint8_t*)TxStr, strlen(TxStr));
}

  FATFS FatFs;
  FIL Fil;
  FRESULT FR_Status;
  FATFS *FS_Ptr;
  UINT RWC, WWC; 
  DWORD FreeClusters;
  uint32_t TotalSize, FreeSpace;
  char RW_Buffer[200];
  volatile uint8_t statusMode = 0;
  volatile uint8_t sd_busy = 0;
  enum status {
    READ_NOT_READY = 0,
    READ_READY = 1,
  };
int main(void)
{
 
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  if(open_log_file() != FR_OK){
    Error_Handler();
  };
  Enable_Magnetometer();
  while (1){
    if(statusMode == READ_READY){
      SDIO_SDCard_Read(); 
    }
    DMA_Write();
  }
    return 0;
}


static void Enable_Magnetometer(void){
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(100); 
}


float period_to_field(const uint32_t period, const float A, const float B, const float C, const float D) {
    if (period == 0) return 0.0f;
    
    // ВНИМАНИЕ: знак МИНУС перед D!
    float arg = C / (float)period - D;
    
    // Защита от точек разрыва тангенса
    float normalized_arg = fmodf(fabsf(arg), 3.1415926535f);
    if (normalized_arg > 1.4f && normalized_arg < 1.7f) {
        return A;
    }
    
    float tangent_val = tanf(arg);
    
    // Ограничение амплитуды
    if (tangent_val > 100.0f) tangent_val = 100.0f;
    if (tangent_val < -100.0f) tangent_val = -100.0f;
    
    // Округление как в оригинале
    return roundf(A + B * tangent_val);
}

static FRESULT open_log_file(void) {
    FR_Status = f_mount(&FatFs, SDPath, 1);
    if(FR_Status != FR_OK){
        sprintf(TxBuffer, "Mount error: %i\r\n", FR_Status);
        USB_CDC_Print(TxBuffer);
        Error_Handler();
    }
    
    FR_Status = f_open(&Fil, "text.txt", FA_WRITE | FA_CREATE_ALWAYS);
    if (FR_Status != FR_OK) {
        sprintf(TxBuffer, "Open error: %i\r\n", FR_Status);
        USB_CDC_Print(TxBuffer);
        Error_Handler();
    }
    file_is_open = 1;
    return FR_OK;
}


void DMA_Write(void) {
    if (sd_busy) return;
    sd_busy = 1;
    
    if (!file_is_open) {
        FR_Status = f_open(&Fil, "text.txt", FA_WRITE | FA_CREATE_ALWAYS);
        if (FR_Status != FR_OK) {
            sprintf(TxBuffer, "Open for append error: %i\r\n", FR_Status);
            USB_CDC_Print(TxBuffer);
            sd_busy = 0;
            return;
        }
        file_is_open = 1;
    }

        // x = round(Ax_default + Bx_default*tan(Cx_default/(float)periods[0]-Dx_default));
				// y = round(Ay_default + By_default*tan(Cy_default/(float)periods[1]-Dy_default));
				// z = round(Az_default + Bz_default*tan(Cz_default/(float)periods[2]-Dz_default));
				// tfactor = (1 + tc*(t0 - 0.01f));
				// x = x*tfactor;
				// y = y*tfactor;
				// z = z*tfactor;
        // float Hx_true, Hy_true;
				// Hx_true = round(x*Mxx + y*Mxy + z*Mxz);
				// Hy_true = round(x*Myx + y*Myy + z*Myz);
        x = periods [0];
        y = periods [1];
        z = periods [2];

    sprintf(RW_Buffer, "Hx period: %10.3f  Hy period: %10.3f  Hz period: %10.3f\r\n", x, y, z);
    DWORD file_size = f_size(&Fil);
    if (file_size > 1024*10) { 
        f_close(&Fil);
        file_is_open = 0;
        
        FR_Status = f_open(&Fil, "text.txt", FA_WRITE | FA_CREATE_ALWAYS);
        if (FR_Status != FR_OK) {
            sprintf(TxBuffer, "Recreate error: %i\r\n", FR_Status);
            USB_CDC_Print(TxBuffer);
            sd_busy = 0;
            return;
        }
        file_is_open = 1;
    }
    
    FR_Status = f_write(&Fil, RW_Buffer, strlen(RW_Buffer), &WWC);
    if(FR_Status != FR_OK){
        sprintf(TxBuffer, "Write error: %i\r\n", FR_Status);
        USB_CDC_Print(TxBuffer);
    }
    
    f_sync(&Fil);
    sd_busy = 0;
}

void SDIO_SDCard_Read(void) {
    if (sd_busy) return;
    sd_busy = 1;

    if (file_is_open) {
        f_close(&Fil);
        file_is_open = 0;
        HAL_Delay(10); 
    }

    FR_Status = f_open(&Fil, "text.txt", FA_READ);
    if (FR_Status != FR_OK) {
        sprintf(TxBuffer, "Open for read error: %i\r\n", FR_Status);
        USB_CDC_Print(TxBuffer);
        sd_busy = 0;
        return;
    }
    file_is_open = 1;

    DWORD file_size = f_size(&Fil);
    if (file_size == 0) {
        sprintf(TxBuffer, "File is empty.\r\n");
        USB_CDC_Print(TxBuffer);
        f_close(&Fil);
        file_is_open = 0;
        sd_busy = 0;
        return;
    }

    char chunk[256];
    UINT bytes_read;
    DWORD bytes_left = file_size;

    while (bytes_left > 0) {
        UINT chunk_size = (bytes_left > sizeof(chunk)-1) ? sizeof(chunk)-1 : bytes_left;
        FR_Status = f_read(&Fil, chunk, chunk_size, &bytes_read);
        if (FR_Status != FR_OK || bytes_read == 0) {
            break;
        }

        chunk[bytes_read] = '\0';
        USB_CDC_Print(chunk);
        bytes_left -= bytes_read;
    }

   
    f_close(&Fil);
    file_is_open = 0;
    
    statusMode = READ_NOT_READY;
    sd_busy = 0;
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */

static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */
  __HAL_RCC_TIM2_CLK_ENABLE();
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 5;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  
  // Запускаем таймер и каналы захвата
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);  // Запуск с прерываниями
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2); 
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}


void TIM2_IRQHandler(void){
    uint32_t current_capture = 0x00;
    if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC1)){
      __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);
      current_capture = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
      if(last_capture[0] != 0){
        periods[0] = current_capture - last_capture[0];
      }
      last_capture[0] = current_capture;
    }
      if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC2)){
      __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC2);
      current_capture = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
      if(last_capture[1] != 0){
        periods[1] = current_capture - last_capture[1];
      }
      last_capture[1] = current_capture;
    }
    if(__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC3)){
      __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC3);
      current_capture = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3);
      if(last_capture[2] != 0){
        periods[2] = current_capture - last_capture[2];
      }
      last_capture[2] = current_capture;
    }
}








/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 200;
  if(HAL_SD_Init(&hsd)!= HAL_OK){
    Error_Handler();
  }
  if(HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B)!= HAL_OK){
    Error_Handler();
  }
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

   __HAL_RCC_DMA2_CLK_ENABLE();

  hdma_sdio_rx.Instance = DMA2_Stream3;
  hdma_sdio_rx.Init.Channel = DMA_CHANNEL_4;
  hdma_sdio_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_sdio_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_sdio_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_sdio_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_sdio_rx.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_sdio_rx.Init.Mode = DMA_PFCTRL;
  hdma_sdio_rx.Init.Priority = DMA_PRIORITY_LOW;
  hdma_sdio_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  hdma_sdio_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_sdio_rx.Init.MemBurst = DMA_MBURST_INC4;
  hdma_sdio_rx.Init.PeriphBurst = DMA_PBURST_INC4;
  if (HAL_DMA_Init(&hdma_sdio_rx) != HAL_OK){
    Error_Handler();
  }
  __HAL_LINKDMA(&hsd,hdmarx,hdma_sdio_rx);

  hdma_sdio_tx.Instance = DMA2_Stream6;
  hdma_sdio_tx.Init.Channel = DMA_CHANNEL_4;
  hdma_sdio_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_sdio_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_sdio_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_sdio_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_sdio_tx.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_sdio_tx.Init.Mode = DMA_PFCTRL;
  hdma_sdio_tx.Init.Priority = DMA_PRIORITY_LOW;
  hdma_sdio_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_sdio_tx.Init.MemBurst = DMA_MBURST_INC4;
  hdma_sdio_tx.Init.PeriphBurst = DMA_PBURST_INC4;
  if (HAL_DMA_Init(&hdma_sdio_tx) != HAL_OK){
    Error_Handler();
  }
  __HAL_LINKDMA(&hsd,hdmatx,hdma_sdio_tx);

  /* DMA interrupt init */
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

void DMA2_Stream3_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_sdio_rx);
}
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_SDIO_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
   GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Светодиод */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);


  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; 
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); //x

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);; //y
  
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); //z


  /* Конфигурация линий SDIO */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  /* SDIO CLK - PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* SDIO CMD - PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* SDIO D0 - PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* SDIO D1 - PC9 (опционально для 4-битного режима) */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* SDIO D2 - PC10 (опционально для 4-битного режима) */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* SDIO D3 - PC11 (опционально для 4-битного режима) */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}
void EXTI9_5_IRQHandler(void){
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
}

 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
   if (GPIO_Pin == GPIO_PIN_7)
    {
       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
       USB_CDC_Print("___________________________________________________________________\r\n");
       statusMode = READ_READY;
       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
    HAL_Delay(1000);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
