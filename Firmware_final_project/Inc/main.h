/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2026 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define oECATErrorLed_Pin GPIO_PIN_2
#define oECATErrorLed_GPIO_Port GPIOE
#define ECAT_IRQ_Pin GPIO_PIN_0
#define ECAT_IRQ_GPIO_Port GPIOC
#define ECAT_IRQ_EXTI_IRQn EXTI0_IRQn
#define ECAT_SYNC0_Pin GPIO_PIN_1
#define ECAT_SYNC0_GPIO_Port GPIOC
#define ECAT_SYNC0_EXTI_IRQn EXTI1_IRQn
#define ECAT_SYNC1_Pin GPIO_PIN_2
#define ECAT_SYNC1_GPIO_Port GPIOC
#define ECAT_SYNC1_EXTI_IRQn EXTI2_IRQn
#define i15V_State_Pin GPIO_PIN_4
#define i15V_State_GPIO_Port GPIOA
#define iIPM_Fault_Pin GPIO_PIN_5
#define iIPM_Fault_GPIO_Port GPIOA
#define iFanConnect_Pin GPIO_PIN_6
#define iFanConnect_GPIO_Port GPIOA
#define iEncFaultIsr_Pin GPIO_PIN_11
#define iEncFaultIsr_GPIO_Port GPIOF
#define iMeasureIsr_Pin GPIO_PIN_13
#define iMeasureIsr_GPIO_Port GPIOF
#define iMeasureIsr_EXTI_IRQn EXTI15_10_IRQn
#define oFaultACK_Pin GPIO_PIN_14
#define oFaultACK_GPIO_Port GPIOF
#define oEnableReadADC_Pin GPIO_PIN_0
#define oEnableReadADC_GPIO_Port GPIOG
#define oDynamicBrake_Pin GPIO_PIN_13
#define oDynamicBrake_GPIO_Port GPIOB
#define iBrakeResConnect_Pin GPIO_PIN_14
#define iBrakeResConnect_GPIO_Port GPIOB
#define oResetEncoder_Pin GPIO_PIN_2
#define oResetEncoder_GPIO_Port GPIOG
#define iFPGA_DONE_Pin GPIO_PIN_9
#define iFPGA_DONE_GPIO_Port GPIOA
#define iUSB_VBUS_Pin GPIO_PIN_10
#define iUSB_VBUS_GPIO_Port GPIOA
#define oOutput4_Pin GPIO_PIN_2
#define oOutput4_GPIO_Port GPIOD
#define iInput6_Pin GPIO_PIN_3
#define iInput6_GPIO_Port GPIOD
#define oOutput3_Pin GPIO_PIN_6
#define oOutput3_GPIO_Port GPIOD
#define EC_CS_Pin GPIO_PIN_7
#define EC_CS_GPIO_Port GPIOD
#define FPGA_CS_Pin GPIO_PIN_9
#define FPGA_CS_GPIO_Port GPIOG
#define iInput5_Pin GPIO_PIN_10
#define iInput5_GPIO_Port GPIOG
#define oOutput2_Pin GPIO_PIN_11
#define oOutput2_GPIO_Port GPIOG
#define iInput4_Pin GPIO_PIN_12
#define iInput4_GPIO_Port GPIOG
#define oOutput1_Pin GPIO_PIN_13
#define oOutput1_GPIO_Port GPIOG
#define iInput3_Pin GPIO_PIN_14
#define iInput3_GPIO_Port GPIOG
#define iInput2_Pin GPIO_PIN_15
#define iInput2_GPIO_Port GPIOG
#define iInput1_Pin GPIO_PIN_5
#define iInput1_GPIO_Port GPIOB
#define iUSB_OverCurrent_Pin GPIO_PIN_6
#define iUSB_OverCurrent_GPIO_Port GPIOB
#define oDriveErrorLed_Pin GPIO_PIN_7
#define oDriveErrorLed_GPIO_Port GPIOB
#define oDriveRunLed_Pin GPIO_PIN_8
#define oDriveRunLed_GPIO_Port GPIOB
#define oTLED3_Pin GPIO_PIN_9
#define oTLED3_GPIO_Port GPIOB
#define oTLED2_Pin GPIO_PIN_0
#define oTLED2_GPIO_Port GPIOE
#define oTLED1_Pin GPIO_PIN_1
#define oTLED1_GPIO_Port GPIOE

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
