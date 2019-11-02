/*-------------------------------------------------------------------------- *
 *                                                                           * 
 *                                                                           *
 *               ____                     _____                              *
 *              / ___|  ___ __ _ _ __ ___|  ___|   _ _ __                    *
 *              \___ \ / __/ _` | '__/ __| |_ | | | | '_ \                   *
 *               ___) | (_| (_| | |  \__ \  _|| |_| | | | |                  *
 *              |____/ \___\__,_|_|  |___/_|   \__,_|_| |_|                  *
 *                                                                           *
 *                      Dirty STM32 I2C Scanner V0.01                        *
 *   02/10/2019                                                              * 
 *                                                                           * 
 *                                                                           * 
 *   https://github.com/ScarsFun                                             * 
 *                                                                           * 
 *---------------------------------------------------------------------------*/                       

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include <stdio.h>

#define I2C_REQUEST_WRITE 0x00

char CDC_tx_buff[64];
char CDC_rx_buff[8];
uint8_t CDC_rx_flag = 0;
uint32_t I2C_speed[4] = { 100000, 200000, 300000, 400000 };


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

uint8_t I2C_Check(uint16_t addr)
{

    uint8_t timeOut = 10;
    LL_I2C_DisableBitPOS(I2C1);
    LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
    LL_I2C_GenerateStartCondition(I2C1);
    while (!LL_I2C_IsActiveFlag_SB(I2C1)) {
        if (LL_SYSTICK_IsActiveCounterFlag()) {
            if (timeOut-- == 0) {
                LL_I2C_GenerateStopCondition(I2C1);
                return 0;
            }
        }
    }
    LL_I2C_TransmitData8(I2C1, (addr) << 1 | I2C_REQUEST_WRITE);
    while (!LL_I2C_IsActiveFlag_ADDR(I2C1)) {
        if (LL_SYSTICK_IsActiveCounterFlag()) {
            if (timeOut-- == 0) {
                LL_I2C_GenerateStopCondition(I2C1);
                return 0;
            }
        }
    }
    LL_I2C_ClearFlag_ADDR(I2C1);
    LL_I2C_GenerateStopCondition(I2C1);
    return 1;
}

uint8_t I2C_Roll_Speed(uint16_t addr)
{
    LL_RCC_ClocksTypeDef rcc_clocks;
    uint8_t idx;
    uint32_t freq;

    LL_RCC_GetSystemClocksFreq(&rcc_clocks);
    sprintf(CDC_tx_buff, "\r\n0X%x", addr);
    CDC_Transmit_FS(CDC_tx_buff, 7);
    for (idx = 0; idx < 4; idx++) {
        LL_mDelay(3);
        LL_I2C_Disable(I2C1);
        LL_I2C_ConfigSpeed(I2C1, rcc_clocks.PCLK1_Frequency, I2C_speed[idx], LL_I2C_DUTYCYCLE_2);
        LL_I2C_Enable(I2C1);
        if (I2C_Check(addr)) {
            sprintf(CDC_tx_buff, "   V   ");
            CDC_Transmit_FS(CDC_tx_buff, 7);
            LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12); // LED on : I2C slave found
        }
        else {
            sprintf(CDC_tx_buff, "  ---  ");
            CDC_Transmit_FS(CDC_tx_buff, 7);
        }
    }
    return 0;
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USB_DEVICE_Init();

    LL_mDelay(5000);
    while (1) {
        LL_mDelay(1000);
        CDC_Transmit_FS("\n\rDirty STM32 I2C Scanner V0.01\r\ntype 's' to scan\n\r", 51);
        while (!CDC_rx_flag) {
            LL_mDelay(1000);
        }
        if (CDC_rx_flag == 1) {
						LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12); 
            CDC_Transmit_FS("SCAN...\n\r", 9);
            LL_mDelay(5);
            CDC_Transmit_FS("     100Khz 200Khz 300Khz 400Khz", 35);
            for (int i = 1; i < 128; i++) {
                LL_mDelay(100);
                I2C_Roll_Speed(i);
            }
        }
        CDC_rx_flag = 0;
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

    if (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1) {
        Error_Handler();
    }
    LL_RCC_HSE_Enable();

    /* Wait till HSE is ready */
    while (LL_RCC_HSE_IsReady() != 1) {
    }
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_6);
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1) {
    }
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
    }
    LL_SetSystemCoreClock(48000000);
    LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_PLL);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

    LL_I2C_InitTypeDef I2C_InitStruct = { 0 };

    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
    /**I2C1 GPIO Configuration  
  PB6   ------> I2C1_SCL
  PB7   ------> I2C1_SDA 
  */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    /** I2C Initialization 
  */
    LL_I2C_DisableOwnAddress2(I2C1);
    LL_I2C_DisableGeneralCall(I2C1);
    LL_I2C_EnableClockStretching(I2C1);
    I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
    I2C_InitStruct.ClockSpeed = 100000;
    I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
    I2C_InitStruct.OwnAddress1 = 0;
    I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
    I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
    LL_I2C_Init(I2C1, &I2C_InitStruct);
    LL_I2C_SetOwnAddress2(I2C1, 0);
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    /* GPIO Ports Clock Enable */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
	  
	// STM32 'black pill' onboard LED
  

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);
	
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
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
