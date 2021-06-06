#define BOOTSTUB

#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio_ex.h"

#include <stdbool.h>

#if 0
void puts(const char *a){ UNUSED(a); }
void puth(unsigned int i){ UNUSED(i); }
void puth2(unsigned int i){ UNUSED(i); }
#endif

// ********************* Includes *********************
#include <stddef.h>
#include "libc.h"
#include "critical.h"
#include "faults.h"
#include "registers.h"
#include "interrupts.h"
#include "clock.h"
#include "llgpio.h"
#include "uart.h"



#define LED_GPIO  GPIOA
#define LED_BLUE  1
#define LED_GREEN 2
#define LED_RED   3

#define LED_ON    0
#define LED_OFF   1

#define LED_DELAY 5000000

int led_value = 0;

void led_set(int led, int value);

void debug_ring_callback(uart_ring *ring) {
      char rcv;
      static int led_val = 0;
      while (getc(ring, &rcv) != 0) {
              (void)putc(ring, rcv);
      }
      led_set(LED_BLUE, led_val);
      led_val = !led_val;
      
}

void TIM3_IRQ_Handler(void) {

   // blink the LED
   led_set(LED_RED, led_value);
   led_value = !led_value;
   TIM3->SR = 0;
          
}

void timer_init(TIM_TypeDef *TIM, int psc) {
      register_set(&(TIM->PSC), (psc-1), 0xFFFFU);
      register_set(&(TIM->DIER), TIM_DIER_UIE, 0x5F5FU);
      register_set(&(TIM->CR1), TIM_CR1_CEN, 0x3FU);
      TIM->SR = 0;
}


void led_set(int led, int value)
{
    switch (led) {
    case LED_BLUE:
            set_gpio_mode(LED_GPIO, 5, !value);
            break;
    case LED_GREEN:
            set_gpio_mode(LED_GPIO, 6, !value);
            break;
    case LED_RED:
            set_gpio_mode(LED_GPIO, 7, !value);
            break;
    default:
            break;
                       
    }
}

void led_init(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

  set_gpio_mode(GPIOA, 5, MODE_OUTPUT);
  set_gpio_mode(GPIOA, 6, MODE_OUTPUT);
  set_gpio_mode(GPIOA, 7, MODE_OUTPUT);

  led_set(LED_BLUE,  LED_OFF);
  led_set(LED_GREEN, LED_OFF);
  led_set(LED_RED,   LED_OFF);
  
}

void timer3_init(void)
{
  register_set_bits(&(RCC->APB1ENR), RCC_APB1ENR_TIM3EN); 
  timer_init(TIM3, 15);
  NVIC_EnableIRQ(TIM3_IRQn);
}

int main(void) {

  int counter;
  volatile unsigned int id = DBGMCU->IDCODE;

  // Init interrupt table
  init_interrupts(true);
  REGISTER_INTERRUPT(TIM3_IRQn, TIM3_IRQ_Handler, 1000U, FAULT_INTERRUPT_RATE_TIM3);

  disable_interrupts();
  clock_init();
  led_init();
  timer3_init();

  // USART2 
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  set_gpio_alternate(GPIOD, 5, GPIO_AF7_USART2);
  set_gpio_alternate(GPIOD, 6, GPIO_AF7_USART2);
  uart_init(&uart_ring_debug, 115200);

  enable_interrupts();

  puts("MCU_ID:"); puth(id); puts("\n");

  counter = 0;
  for(;;){
      led_set(LED_GREEN, LED_OFF);
      delay(LED_DELAY);
      led_set(LED_GREEN, LED_ON);
      delay(LED_DELAY);
      puts("counter:"); puth(counter); puts("\n");
      counter++;
  }

  return 0;
}

