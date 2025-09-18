/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
  /*
   __  __          _     _            _                _                    _           
   \ \/ /   _  ___| |   (_)_ __      / \   _ __  _ __ | |    ___   __ _  __| | ___ _ __ 
    \  / | | |/ _ \ |   | | '_ \    / _ \ | '_ \| '_ \| |   / _ \ / _` |/ _` |/ _ \ '__|
    /  \ |_| |  __/ |___| | | | |  / ___ \| |_) | |_) | |__| (_) | (_| | (_| |  __/ |   
   /_/\_\__,_|\___|_____|_|_| |_| /_/   \_\ .__/| .__/|_____\___/ \__,_|\__,_|\___|_|   
                                          |_|   |_|                                     
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "quadspi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <sys/_intsup.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_VALID 0
#define APP_INVALID 1
// #define RX_DATA_SIZE 16
#define QSPI_APP_ADDRESS W25Qxx_Mem_Addr
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t QSPI_Status = 0;
uint32_t Device_ID;	    // Flash ID
const char ver[] = "0.1.0.10";
// uint8_t ReceiveData[RX_DATA_SIZE] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
uint8_t Validate_Stack_Pointer(uint32_t stack_pointer);
uint8_t Validate_Application(void);
void JumpToApplication(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Print_LOGO(void)
{
  printf("\r\n\033[36m");
  printf(" __  __          _     _            _                _                    _           \n");
  printf(" \\ \\/ /   _  ___| |   (_)_ __      / \\   _ __  _ __ | |    ___   __ _  __| | ___ _ __ \n");
  printf("  \\  / | | |/ _ \\ |   | | '_ \\    / _ \\ | '_ \\| '_ \\| |   / _ \\ / _` |/ _` |/ _ \\ '__|\n");
  printf("  /  \\ |_| |  __/ |___| | | | |  / ___ \\| |_) | |_) | |__| (_) | (_| | (_| |  __/ |   \n");
  printf(" /_/\\_\\__,_|\\___|_____|_|_| |_| /_/   \\_\\ .__/| .__/|_____\\___/ \\__,_|\\__,_|\\___|_|   \n");
  printf("                                        |_|   |_|                                     \n");
  printf("\033[0m");
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_QUADSPI_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // HAL_UART_Receive_IT(&huart1, ReceiveData, RX_DATA_SIZE);

  printf("\033[36m========XueLin AppLoader========\n          version:%s\n\033[34mApplication base address:0x%x\033[0m\n", ver, QSPI_APP_ADDRESS);
  Print_LOGO();
	
	QSPI_W25Qxx_Reset();
	Device_ID = QSPI_W25Qxx_ReadID();
  if (Device_ID != W25Qxx_FLASH_ID)
  {
      printf("\033[31mFlash Error or Broken!\033[0m\n");
      return 0;
  }
  else {
    printf("\033[32mFlash OK!\033[0m\n");
    QSPI_Status = QSPI_W25Qxx_MemoryMappedMode();
    if (QSPI_Status != QSPI_W25Qxx_OK)
    {
      printf("\033[31mSet QSPI Flash to Memory Map mode has some problem!\nError Code: %ld\033[0m\n", QSPI_Status);
      return 0;
    }
    else {
      printf("\033[32mQPSI Flash set Memory Mode OK\033[0m\n");
      if(Validate_Application() != APP_VALID)
       {
        // 应用程序无效，可进入编程模式
        // Enter_Program_Mode();
        printf("\033[31mApp Invalid!\033[0m\n");
        return 0;
      }
      else {
        printf("\033[32mApp Vaild! Will run it!\033[0m\n");
        JumpToApplication();
      }
      
    }  
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*校验应用程序栈指针*/
uint8_t Validate_Application(void)
{
    // // 检查应用程序起始地址的栈指针是否有效
    uint32_t* app_stack_ptr = (uint32_t*)QSPI_APP_ADDRESS;
    if (!Validate_Stack_Pointer(*app_stack_ptr))
    {
      printf("\033[31mInvalid stack pointer: 0x%lx\033[0m\n", *app_stack_ptr);
      return APP_INVALID;
    }
    
    // 检查复位向量是否在QSPI地址范围内
    uint32_t* app_reset_handler = (uint32_t*)(QSPI_APP_ADDRESS + 4);
    if((*app_reset_handler & 0xFF000000) != 0x90000000)
    {
        return APP_INVALID;
    }
    
    return APP_VALID;
}

/*校验堆栈指针范围*/
uint8_t Validate_Stack_Pointer(uint32_t stack_pointer)
{
    // 检查栈指针是否指向有效的RAM区域或特定保留区域
    // STM32H750的RAM区域包括：
    // - DTCM RAM: 0x20000000 - 0x2001FFFF (128KB)
    // - AXI SRAM: 0x24000000 - 0x2407FFFF (512KB)
    // - AHB SRAM: 0x30000000 - 0x30047FFF (288KB)
    // - AHB SRAM: 0x38000000 - 0x3800FFFF (64KB)
    // STM32H750的特定保留区域包括：
    // - Reseved0: 0x20002000 - 0x23FFFFFF
    // - Reseved1: 0x24080000 - 0x2FFFFFFF
    // - Reseved2: 0x30048000 - 0x37FFFFFF
    // - Reseved3: 0x38001000 - 0x387FFFFF
    
    if ((stack_pointer >= 0x20000000 && stack_pointer <= 0x2001FFFF) ||  // DTCM RAM
        (stack_pointer >= 0x20002000 && stack_pointer <= 0x23FFFFFF) ||  // Reseved0
        (stack_pointer >= 0x24000000 && stack_pointer <= 0x2407FFFF) ||  // AXI SRAM
        (stack_pointer >= 0x24080000 && stack_pointer <= 0x2FFFFFFF) ||  // Reseved1
        (stack_pointer >= 0x30000000 && stack_pointer <= 0x30047FFF) ||  // AHB SRAM
        (stack_pointer >= 0x30048000 && stack_pointer <= 0x37FFFFFF) ||  // Reseved2
        (stack_pointer >= 0x38000000 && stack_pointer <= 0x3800FFFF) ||  // AHB SRAM
        (stack_pointer >= 0x38001000 && stack_pointer <= 0x387FFFFF))    // Reseved3
    {
        return 1; // 有效
    }
    
    return 0; // 无效
}

/* 跳转到应用程序 */
void JumpToApplication(void)
{
    typedef void (*pFunction)(void);
    pFunction Jump_To_App;
    
    // 禁用所有中断
    __disable_irq();
    
    // 设置应用程序的向量表地址
    SCB->VTOR = QSPI_APP_ADDRESS;
    
    // 初始化应用程序的栈指针
    __set_MSP(*(__IO uint32_t*)QSPI_APP_ADDRESS);
    
    // 获取应用程序的复位处理函数地址
    Jump_To_App = (pFunction)(*(__IO uint32_t*)(QSPI_APP_ADDRESS + 4));

    // 跳转到应用程序
    Jump_To_App();
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//   if(huart->Instance == USART1)
//   {
//     QSPI_W25Qxx_WriteBuffer(ReceiveData, 0x00000000, RX_DATA_SIZE);
//     HAL_UART_Transmit(&huart1, ReceiveData, RX_DATA_SIZE, HAL_MAX_DELAY);
//     // memset(ReceiveData, 0x00, RX_DATA_SIZE);
//     HAL_UART_Receive_IT(&huart1, ReceiveData, RX_DATA_SIZE);
//   }
// }
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x20000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_128KB;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x24000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
  MPU_InitStruct.SubRegionDisable = 0x0;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
