/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MXCONSTANT_H
#define __MXCONSTANT_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define EXTI13_MAG_RDY_Pin GPIO_PIN_13
#define EXTI13_MAG_RDY_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOC
#define USART2_TX_XBee_Pin GPIO_PIN_2
#define USART2_TX_XBee_GPIO_Port GPIOA
#define USART2_RX_XBee_Pin GPIO_PIN_3
#define USART2_RX_XBee_GPIO_Port GPIOA
#define TIM3_CH3_PWM_Pin GPIO_PIN_0
#define TIM3_CH3_PWM_GPIO_Port GPIOB
#define TIM3_CH4_PWM_Pin GPIO_PIN_1
#define TIM3_CH4_PWM_GPIO_Port GPIOB
#define EXTI2_SafeBut_Pin GPIO_PIN_2
#define EXTI2_SafeBut_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOB
#define TIM3_CH1_PWM_Pin GPIO_PIN_6
#define TIM3_CH1_PWM_GPIO_Port GPIOC
#define TIM3_CH2_PWM_Pin GPIO_PIN_7
#define TIM3_CH2_PWM_GPIO_Port GPIOC
#define TIM8_CH3_PPM_Pin GPIO_PIN_8
#define TIM8_CH3_PPM_GPIO_Port GPIOC
#define USB_PP_Pin GPIO_PIN_10
#define USB_PP_GPIO_Port GPIOA
#define USB_OUT_Pin GPIO_PIN_10
#define USB_OUT_GPIO_Port GPIOC
#define USB_IN_Pin GPIO_PIN_11
#define USB_IN_GPIO_Port GPIOC
#define USART5_TX_IBUS_Pin GPIO_PIN_12
#define USART5_TX_IBUS_GPIO_Port GPIOC
#define USART5_RX_IBUS_Pin GPIO_PIN_2
#define USART5_RX_IBUS_GPIO_Port GPIOD
#define USART1_TX_GPS_Pin GPIO_PIN_6
#define USART1_TX_GPS_GPIO_Port GPIOB
#define USART1_RX_GPS_Pin GPIO_PIN_7
#define USART1_RX_GPS_GPIO_Port GPIOB
#define I2C1_SCL_MAG_Pin GPIO_PIN_8
#define I2C1_SCL_MAG_GPIO_Port GPIOB
#define I2C1_SDA_MAG_Pin GPIO_PIN_9
#define I2C1_SDA_MAG_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MXCONSTANT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
