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

#include "stm32l0xx.h"
#include "usblib.h"

void SetClocks();
void Delay(uint32_t delay);
USBLIB_WByte _LineState;
uint32_t stick = 0;

int main(void)
{
    SetClocks();
    // set SysTick = 1ms
    SysTick_Config(32000);

    // ===== GPIO init =====
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;
    // output mode (01) for PB6 PB7 - debug signal out
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE6|GPIO_MODER_MODE7)) | (GPIO_MODER_MODE6_0|GPIO_MODER_MODE7_0);
    GPIOA->OSPEEDR = GPIOA->OSPEEDR | GPIO_OSPEEDER_OSPEED6 | GPIO_OSPEEDER_OSPEED7; // very high speed

    // output mode (01) for PBA PB8 - LED pins
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE4)) | (GPIO_MODER_MODE4_0);
    GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODE8)) | (GPIO_MODER_MODE8_0);

    // =========== TIM2 ==========
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 1000 - 1;
    TIM2->ARR = 32000 - 1;
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
    NVIC_SetPriority(TIM2_IRQn, 15);
    NVIC_EnableIRQ(TIM2_IRQn);

    USBLIB_Init();

#pragma clang diagnostic ignored "-Wmissing-noreturn"
    while (1) {
        GPIOA->ODR ^= GPIO_ODR_OD4;
        Delay(300);
    }
}

void SetClocks() {
// ==============  HSI
    RCC->CR |= RCC_CR_HSION;
    while (RCC->CR & RCC_CR_HSIRDY == RESET);

// ============== HSI48
    SET_BIT(RCC->CRRCR, RCC_CRRCR_HSI48ON);
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);
    while (RCC->CRRCR & RCC_CRRCR_HSI48RDY == RESET);

// ============== PLL
    RCC->CFGR |= RCC_CFGR_PLLSRC_HSI | RCC_CFGR_PLLMUL4 | RCC_CFGR_PLLDIV2;
    // Enable the main PLL.
    SET_BIT(RCC->CR, RCC_CR_PLLON);
    // Wait till PLL is ready
    while (RCC->CR & RCC_CR_PLLRDY == RESET);

// ============== FLASH
    FLASH->ACR |= FLASH_ACR_LATENCY;
    // Check that the new number of wait states is taken into account to access the Flash
    // memory by reading the FLASH_ACR register
    while ((FLASH->ACR & FLASH_ACR_LATENCY) == RESET);

// ============== HCLK
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

// ============== SYSCLK
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while (RCC->CFGR & RCC_CFGR_SWS != RCC_CFGR_SWS_PLL);

    // set HSI48 as USB Clock source
    RCC->CCIPR |= RCC_CCIPR_HSI48SEL;

// ==========  RCCEx_CRSConfig

    // Before configuration, reset CRS registers to their default values
    // RCC_CRS_FORCE_RESET()
    SET_BIT(RCC->APB1RSTR, RCC_APB1RSTR_CRSRST);
    // RCC_CRS_RELEASE_RESET()
    CLEAR_BIT(RCC->APB1RSTR, RCC_APB1RSTR_CRSRST);

// Set the SYNCDIV[2:0] bits to Prescaler value
#define RCC_CRS_SYNC_DIV1              ((uint32_t)0x00000000U) // Synchro Signal not divided (default)
// Set the SYNCSRC[1:0] bits to Source value
#define RCC_CRS_SYNC_SOURCE_USB        CRS_CFGR_SYNCSRC_1      // Synchro Signal source USB SOF (default)
// Set the SYNCSPOL bit to Polarity value
#define RCC_CRS_SYNC_POLARITY_RISING   ((uint32_t)0x00000000U) // Synchro Active on rising edge (default)
// Set the RELOAD[15:0] bits to ReloadValue value
#define RCC_CRS_RELOADVALUE_CALCULATE(__FTARGET__, __FSYNC__)  (((__FTARGET__) / (__FSYNC__)) - 1)
// Set the FELIM[7:0] bits according to ErrorLimitValue value
#define RCC_CRS_ERROR_LIMIT 34

    CRS->CFGR = RCC_CRS_SYNC_DIV1 | RCC_CRS_SYNC_SOURCE_USB | RCC_CRS_SYNC_POLARITY_RISING |
                RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000) | (RCC_CRS_ERROR_LIMIT << CRS_CFGR_FELIM_Pos);

    // Adjust HSI48 oscillator smooth trimming
    // Set the TRIM[5:0] bits according to RCC_CRS_HSI48CalibrationValue value
    //MODIFY_REG(CRS->CR, CRS_CR_TRIM, (32 << CRS_CR_TRIM_Pos));

    // Enable Automatic trimming & Frequency error counter
    CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;
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
    GPIOB->ODR ^= GPIO_ODR_OD8;

//    if (_LineState.L) {      // App connected to the virtual port
//        USBLIB_Transmit((uint16_t *)"Welcome to the club!\r\n", 22);
//    }
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
