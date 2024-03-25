/**
 *
 * Monthon Paul
 * u1274364
 *
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"
void _Error_Handler(char *file, int line);

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int flag;
char color, mode;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/**
 * @brief Read a character to Transmit Data Register
 * @param c character it reads
*/
void transmit_char(char c) {
    while (!(USART3->ISR & USART_ISR_TXE)) {
    }
    USART3->TDR = c;
}

/**
 * @brief Read a string to Transmit Data Register
 * @param s a C string
*/
void transmit_string(char *s) {
    for (int i = 0; s[i] != '\0'; i++)
        transmit_char(s[i]);
}

/**
 * @brief Toggle LEDS from given r,g,b,o characters 
 *        4.1 section checkoff
*/
void recieve_LED() {
    if (USART3->ISR & USART_CR1_RXNEIE) {
        color = USART3->RDR;
        // transmit_string("recieve char");
        switch (color) {
            case 'r':
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
                break;
            case 'b':
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
                break;
            case 'o':
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
                break;
            case 'g':
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
                break;
        }
        if (color != 'r' && color != 'b' && color != 'g' && color != 'o')
            transmit_string("Error");
    }
}

/**
 * @brief Toggle LEDS from given r,g,b,o characters with extra modes
 *        4.2 section checkoff
*/
void USART3_4_IRQHandler() {
    if (!flag) {
        color = USART3->RDR;
        flag = 1;
    } else {
        switch (color) {
            case 'r':
                mode = USART3->RDR;
                if (mode == '0')
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
                else if (mode == '1')
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
                else if (mode == '2')
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
                break;
            case 'b':
                mode = USART3->RDR;
                if (mode == '0')
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
                else if (mode == '1')
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
                else if (mode == '2')
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
                break;
            case 'o':
                mode = USART3->RDR;
                if (mode == '0')
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
                else if (mode == '1')
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
                else if (mode == '2')
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
                break;
            case 'g':
                mode = USART3->RDR;
                if (mode == '0')
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
                else if (mode == '1')
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
                else if (mode == '2')
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
                break;
        }
        flag = 0;
    }
    if (color != 'r' && color != 'b' && color != 'g' && color != 'o')
        transmit_string("Error");
}
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
// Sine Wave: 8-bit, 32 samples/cycle
const uint8_t sine_table[32] = {127,151,175,197,216,232,244,251,254,251,244,
232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};
// Triangle Wave: 8-bit, 32 samples/cycle
const uint8_t triangle_table[32] = {0,15,31,47,63,79,95,111,127,142,158,174,
190,206,222,238,254,238,222,206,190,174,158,142,127,111,95,79,63,47,31,15};
// Sawtooth Wave: 8-bit, 32 samples/cycle
const uint8_t sawtooth_table[32] = {0,7,15,23,31,39,47,55,63,71,79,87,95,103,
111,119,127,134,142,150,158,166,174,182,190,198,206,214,222,230,238,246};
// Square Wave: 8-bit, 32 samples/cycle
const uint8_t square_table[32] = {254,254,254,254,254,254,254,254,254,254,
254,254,254,254,254,254,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

void print_number(uint16_t num) {
    int hun = num / 100;
    int ten = (num - 100 * hun) / 10;
    int one = num % 10;
    transmit_char(hun + '0');
    transmit_char(ten + '0');
    transmit_char(one + '0');
    transmit_char(';');
    
}

int main(void) {
    HAL_Init();
    SystemClock_Config();

    // Initialize Clock
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    RCC->APB1ENR = RCC_APB1ENR_USART3EN | RCC_APB1ENR_DACEN;

    // Initialize LED pins
    GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_6 | GPIO_PIN_7,
                                GPIO_MODE_OUTPUT_PP,
                                GPIO_SPEED_FREQ_LOW,
                                GPIO_NOPULL};
    HAL_GPIO_Init(GPIOC, &initStr);  // Initialize LED pins

    /* 4.1 section */
    GPIO_InitTypeDef initStr2 = {GPIO_PIN_10 | GPIO_PIN_11,
                                 GPIO_MODE_AF_PP,
                                 GPIO_SPEED_FREQ_LOW,
                                 GPIO_NOPULL};
    HAL_GPIO_Init(GPIOC, &initStr2);

    GPIOC->AFR[1] |= (GPIO_AF1_USART3 << 8) | (GPIO_AF1_USART3 << 12);

    USART3->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_RXNEIE;
    USART3->BRR = 69;  // HAL_RCC_GetHCLKFreq() / 115200 ~= 70 or 69

    NVIC_EnableIRQ(USART3_4_IRQn);

    // IN8 IN9
    GPIO_InitTypeDef adc = {GPIO_PIN_0 | GPIO_PIN_1,
                                 GPIO_MODE_ANALOG,
                                 GPIO_SPEED_FREQ_LOW,
                                 GPIO_NOPULL};
    HAL_GPIO_Init(GPIOC, &adc);

    __HAL_RCC_ADC1_CLK_ENABLE();
    ADC1->CFGR1 |= ADC_CFGR1_CONT;
    // 8
    
    ADC1->CFGR1 |= ADC_CFGR1_RES_1;
    //  hardware triggers disabled (software trigger only).
    ADC1->CFGR1 &= ~(ADC_CFGR1_EXTEN_Msk);
    ADC1->CHSELR |= ADC_CHSELR_CHSEL10;

    if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
    {
    ADC1->CR |= ADC_CR_ADDIS; /* (2) */
    }
    while ((ADC1->CR & ADC_CR_ADEN) != 0)
    {
    /* For robust implementation, add here time-out management */
    }

    ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;

    // Calibration
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL_Msk) {
    }

    // while (1) {
    //     transmit_string("test\n\r");
    // }
    if ((ADC1->ISR & ADC_ISR_ADRDY) != 0) {
        ADC1->ISR |= ADC_ISR_ADRDY;
    }

    ADC1->CR |= ADC_CR_ADEN;

    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (4) */
    {
    /* For robust implementation, add here time-out management */
    }
    ADC1->CR |= ADC_CR_ADSTART;


    // OUT1
    GPIO_InitTypeDef dac = { GPIO_PIN_4,
                                 GPIO_MODE_ANALOG,
                                 GPIO_SPEED_FREQ_LOW,
                                 GPIO_NOPULL};
    HAL_GPIO_Init(GPIOA, &dac);
    // software trigger
    DAC->CR = 7 << DAC_CR_TSEL1_Pos;
    // Enable DAC
    DAC->CR = DAC_CR_EN1;
    

    

    while (1) {
        // HAL_Delay(1000);

        while (!(ADC1->ISR & ADC_ISR_EOC)) {
        }
        uint16_t data = ADC1->DR;
        if (data > 50) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
        }
        if (data > 100) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
        }
        if (data > 150) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
        }
        print_number(data);
        if (data > 200) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
        }

        transmit_string("CMD?\n\r");
        // HAL_Delay(100);
        // while (1) {
            for (int i = 0; i < 32; i++) {
                DAC->DHR8R1 = sine_table[i]; 
                HAL_Delay(1);
                // DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;
            }
        // }
        // recieve_LED();
    }
}

/** System Clock Configuration
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Initializes the CPU, AHB and APB busses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure the Systick interrupt time
     */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    /**Configure the Systick
     */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char *file, int line) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
      ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}

#endif

/**
 * @}
 */

/**
 * @}
 */

