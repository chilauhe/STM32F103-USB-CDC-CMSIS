/* 
 * This file is part of the SaeWave RemoteSwitch (USB-CDC-CMSIS) 
 * distribution (https://github.com/saewave/STM32F103-USB-CDC-CMSIS).
 * Copyright (c) 2017 Samoilov Alexey.
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "stm32f0xx.h"
#include "usblib.h"


void Delay(uint32_t delay);
USBLIB_WByte _LineState;
uint32_t stick = 0;

int main(void)
{
    // set SysTick = 1ms
    SysTick_Config(48000);

    // ===== GPIO init =====
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
    // output mode (01) for PBA PB8 - LED pins
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER4)) | (GPIO_MODER_MODER4_0);
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER8)) | (GPIO_MODER_MODER8_0);

    // =========== TIM2 ==========
    // set period 1 sec
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 1000 - 1;
    TIM2->ARR = 48000 - 1;
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
    NVIC_SetPriority(TIM2_IRQn, 15);
    NVIC_EnableIRQ(TIM2_IRQn);

    USBLIB_Init();

#pragma clang diagnostic ignored "-Wmissing-noreturn"
    while (1) {
        GPIOA->ODR ^= GPIO_ODR_4;
        Delay(300);
    }
}

void HardFault_Handler() {
    while(1);
}

/**
  * @brief  This function handles SysTick Handler.
  */
void SysTick_Handler(void) {
    stick++;
}

/**
 * @param delay in milliseconds
 */
void Delay(uint32_t delay) {
    uint32_t start = stick;
    while (stick - start < delay);
}

void TIM2_IRQHandler() {
    TIM2->SR &= ~TIM_SR_UIF;  // clear the UIF flag
    GPIOB->ODR ^= GPIO_ODR_8;

    if (_LineState.L & 0x01) {      // App connected to the virtual port
        USBLIB_Transmit((uint16_t *) "Welcome to the club!\r\n", 22);
    }
}

void uUSBLIB_DataReceivedHandler(uint16_t *Data, uint16_t Length)
{
    USBLIB_Transmit(Data, Length);
}

void uUSBLIB_LineStateHandler(USBLIB_WByte LineState)
{
//    if (LineState.L)      // App connected to the virtual port
        _LineState = LineState;
}
