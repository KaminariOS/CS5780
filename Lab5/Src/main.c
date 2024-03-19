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
    {
        transmit_char(s[i]);
        if (s[i] == '\n') {
            transmit_char('\r');
        }
    }
}

/**
 * @brief Toggle LEDS from given r,g,b,o characters 
 *        4.1 section checkoff
*/
// void recieve_LED() {
//     if (USART3->ISR & USART_CR1_RXNEIE) {
//         color = USART3->RDR;
//         // transmit_string("recieve char");
//         switch (color) {
//             case 'r':
//                 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
//                 break;
//             case 'b':
//                 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
//                 break;
//             case 'o':
//                 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
//                 break;
//             case 'g':
//                 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
//                 break;
//         }
//         if (color != 'r' && color != 'b' && color != 'g' && color != 'o')
//             transmit_string("Error");
//     }
// }

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

void I2C_write(int addr, char data) {
    // I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
    I2C2->CR2 |= addr << (I2C_CR2_SADD_Pos + 1); 
    I2C2->CR2 |= 1 << I2C_CR2_NBYTES_Pos;
    // Set the RD_WRN bit to indicate a write operation.
    I2C2->CR2 &= ~(I2C_CR2_RD_WRN);
    I2C2->CR2 |= I2C_CR2_START;
    while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))) {
        // HAL_Delay(100);
        // transmit_string("Waiting at the first loop\n");
    }
    if (I2C2->ISR & I2C_ISR_NACKF) {
        transmit_string("NACKF\n");
        // error!        
    }
    I2C2->TXDR = data;
    while (!(I2C2->ISR & I2C_ISR_TC)) {}
}

uint32_t I2C_read(int addr) {
    I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
    I2C2->CR2 |= addr << (I2C_CR2_SADD_Pos + 1); 
    I2C2->CR2 |= 1 << I2C_CR2_NBYTES_Pos;
    I2C2->CR2 |= 1 << I2C_CR2_RD_WRN_Pos;
    I2C2->CR2 |= I2C_CR2_START;
    while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF))) {
    }
    if (I2C2->ISR & I2C_ISR_NACKF) {
        // error!        
    }
    while (!(I2C2->ISR & I2C_ISR_TC)) {}
    return I2C2->RXDR ; 
}

int main(void) {
    HAL_Init();
    SystemClock_Config();

    // Initialize Clock
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

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

 // 2. Set PB11 to alternate function mode, open-drain output typ
 // Set PB13 to alternate function mode, open-drain output type, and select I2C2_SCL as its alternate function.
    GPIO_InitTypeDef initStr_l3 = {GPIO_PIN_11 | GPIO_PIN_13,
                                GPIO_MODE_AF_OD,
                                GPIO_SPEED_FREQ_LOW,
                                GPIO_NOPULL};
    HAL_GPIO_Init(GPIOB, &initStr_l3);
    GPIOB->AFR[1] |= (GPIO_AF1_I2C2 << 12) | (GPIO_AF5_I2C2 << 20);


    // 4. Set PB14 to output mode, push-pull output type, and initialize/set the pin high.
    GPIO_InitTypeDef initStr_l3_1 = {GPIO_PIN_14,
                                GPIO_MODE_OUTPUT_PP,
                                GPIO_SPEED_FREQ_LOW,
                                GPIO_NOPULL};
    HAL_GPIO_Init(GPIOB, &initStr_l3_1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);

    // 5. Set PC0 to output mode, push-pull output type, and initialize/set the pin high.
    GPIO_InitTypeDef initStr_l3_2 = {GPIO_PIN_0,
                                GPIO_MODE_OUTPUT_PP,
                                GPIO_SPEED_FREQ_LOW,
                                GPIO_NOPULL};
    HAL_GPIO_Init(GPIOC, &initStr_l3_2);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

    //  Set the parameters in the TIMINGR register to use 100 kHz s
    I2C2->TIMINGR |= 
        (0x13 << I2C_TIMINGR_SCLL_Pos) | 
        (0xF << I2C_TIMINGR_SCLH_Pos ) |
        (0x2 << I2C_TIMINGR_SDADEL_Pos) | 
        (0x4 << I2C_TIMINGR_SCLDEL_Pos) | 
        (1 << I2C_TIMINGR_PRESC_Pos) ;

    I2C2->CR1 |= I2C_CR1_PE;
    // Set the L3GD20 slave address
    I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
    
    int GYRO_ADDR = 0x69;


    // while (1) {
        I2C_write(GYRO_ADDR, 0xF);

        uint32_t val = I2C_read(GYRO_ADDR);
        if (val == 0xD3) {
            // while (1) {
                transmit_string("Read 0xD3\n");
            // }
        }     
        I2C2->CR2 |= I2C_CR2_STOP;
    // }
    // while (1) {
    //     transmit_string("test 0xD3\n");
    // }

    
    // CTRL_REG1
    I2C_write(GYRO_ADDR, 0x20 | 0xB);

    while (1) {
        HAL_Delay(100);
        transmit_string("Read x_L\n");
        // X_L
        I2C_write(GYRO_ADDR, 0x28);
        char X_L = I2C_read(GYRO_ADDR);
        I2C2->CR2 |= I2C_CR2_STOP;
        // X_H
        transmit_string("Read x_H\n");
        I2C_write(GYRO_ADDR, 0x29);
        char X_H = I2C_read(GYRO_ADDR);
        I2C2->CR2 |= I2C_CR2_STOP;
        // Y_L
        I2C_write(GYRO_ADDR, 0x2A);
        char Y_L = I2C_read(GYRO_ADDR);
        I2C2->CR2 |= I2C_CR2_STOP;
        // Y_H
        I2C_write(GYRO_ADDR, 0x2B);
        char Y_H = I2C_read(GYRO_ADDR);
        I2C2->CR2 |= I2C_CR2_STOP;
            // transmit_string("Loop?");
            // recieve_LED();
        int16_t y_data = (Y_H << 8) | Y_L; 
        int16_t x_data = (X_H << 8) | X_L; 

        /* turning LEDs on*/
        int limit = 10000;
        if (x_data > limit)  // turns on orange LED if X is +
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
        else 
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
        if (x_data < -limit)  // turns on green LED if X is -
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
        else 
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
        if (y_data > limit)  // turns on red LED if Y is +
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
        else 
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
        if (y_data < -limit)  // turns on blue LED if Y is -
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
        else 
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
        // delay for reading
        HAL_Delay(100);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
