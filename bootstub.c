#define BOOTSTUB

#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio_ex.h"

#include <stdbool.h>


void puts(const char *a){ UNUSED(a); }
void puth(unsigned int i){ UNUSED(i); }
void puth2(unsigned int i){ UNUSED(i); }

// ********************* Includes *********************
#include "libc.h"
#include "critical.h"
#include "faults.h"
#include "registers.h"
#include "interrupts.h"
#include "clock.h"
#include "llgpio.h"

#define LED_BLUE  5
#define LED_GREEN 6
#define LED_RED   7

#define LED_ON    0
#define LED_OFF   1

#define LED_DELAY 500000

int led_value = 0;

void TIM3_IRQ_Handler(void) {

   // blink the LED
   set_gpio_output(GPIOA,LED_RED, led_value);
   led_value = !led_value;
   TIM3->SR = 0;
          
}

void timer_init(TIM_TypeDef *TIM, int psc) {
      register_set(&(TIM->PSC), (psc-1), 0xFFFFU);
      register_set(&(TIM->DIER), TIM_DIER_UIE, 0x5F5FU);
      register_set(&(TIM->CR1), TIM_CR1_CEN, 0x3FU);
      TIM->SR = 0;
}



int main(void) {

  // Init interrupt table
  //init_interrupts(true);
  REGISTER_INTERRUPT(TIM3_IRQn, TIM3_IRQ_Handler, 1000U, FAULT_INTERRUPT_RATE_TIM3);

  disable_interrupts();
  clock_init();

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

  //register_set_bits(&(RCC->APB1ENR), RCC_APB1ENR_TIM6EN); 
  register_set_bits(&(RCC->APB1ENR), RCC_APB1ENR_TIM3EN); 

  set_gpio_mode(GPIOA, LED_BLUE,    MODE_OUTPUT);
  set_gpio_mode(GPIOA, LED_GREEN,  MODE_OUTPUT);
  set_gpio_mode(GPIOA, LED_RED,     MODE_OUTPUT);

  set_gpio_output(GPIOA, LED_BLUE,  LED_OFF);
  set_gpio_output(GPIOA, LED_GREEN, LED_OFF);
  set_gpio_output(GPIOA, LED_RED,   LED_OFF);

  timer_init(TIM3, 15);
  NVIC_EnableIRQ(TIM3_IRQn);

  enable_interrupts();

  for(;;){

      set_gpio_output(GPIOA, LED_GREEN, LED_OFF);
      delay(LED_DELAY);
      set_gpio_output(GPIOA, LED_GREEN, LED_ON);
      delay(LED_DELAY);

  }

  return 0;
}

