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
#include "stm32f4xx_hal_gpio_ex.h"
#include "libc.h"
#include "critical.h"
#include "faults.h"
#include "registers.h"
#include "interrupts.h"
#include "clock.h"
#include "llgpio.h"
#include "llcan.h"
#include "uart.h"
#include "can.h"



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

void enable_can_transceiver(uint8_t transceiver, bool enabled) {
   switch (transceiver){
    case 1:
        set_gpio_output(GPIOC, 7, !enabled);
    break;
    case 2:
        set_gpio_output(GPIOC, 9, !enabled);
    break;
    default:
        puts("Invalid CAN transceiver ("); puth(transceiver); puts("): enabling failed\n");
    break;
   }
}

void can_bsp_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    RCC->APB1ENR |= RCC_APB1ENR_CAN2EN;

    set_gpio_mode(GPIOC, 7, MODE_OUTPUT);
    set_gpio_mode(GPIOC, 9, MODE_OUTPUT);
    enable_can_transceiver(1, true);
    enable_can_transceiver(2, true);


    set_gpio_alternate(GPIOB, 8, GPIO_AF9_CAN1);
    set_gpio_alternate(GPIOB, 9, GPIO_AF9_CAN1);
    set_gpio_alternate(GPIOB, 13, GPIO_AF9_CAN2);
    set_gpio_alternate(GPIOB, 5, GPIO_AF9_CAN2);

}

void can_send_test(void *data, int len)
{
  int dpkt = 0;
  CAN_FIFOMailBox_TypeDef to_push;
  uint32_t *d32 = (uint32_t *)data;
  for (dpkt = 0; dpkt < (len / 4); dpkt += 4) {
    to_push.RDHR = d32[dpkt + 3];
    to_push.RDLR = d32[dpkt + 2];
    to_push.RDTR = d32[dpkt + 1];
    to_push.RIR = d32[dpkt];

    to_push.RDTR = (to_push.RDTR & 0xffffff0f) | 1<<4;
    uint8_t bus_number = (to_push.RDTR >> 4) & CAN_BUS_NUM_MASK;
    puts(" --- to_push.RDTR:");
    puth(to_push.RDTR);
    puts(" bus_number: ");
    puth(bus_number);
    can_send(&to_push, bus_number, false);
    puts("\n");
                                    
  }

}


uint8_t send_data[] = { 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0 };

int main(void) {

  int counter;
  bool ret;
  volatile unsigned int id = DBGMCU->IDCODE;

  // Init interrupt table
  init_interrupts(true);
  REGISTER_INTERRUPT(TIM3_IRQn, TIM3_IRQ_Handler, 1000U, FAULT_INTERRUPT_RATE_TIM3);

  disable_interrupts();
  clock_init();
  led_init();
  timer3_init();

  can_bsp_init();
  puts("CANBUS 0: ");
  ret = can_init(CAN_NUM_FROM_BUS_NUM(0));
  puth(ret);
  puts(" CANBUS 1: ");
  ret = can_init(CAN_NUM_FROM_BUS_NUM(1));
  puth(ret);
  puts("\n");
  can_init_all();


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
      can_send_test( can_send, 8);
      counter++;
      
  }

  return 0;
}

