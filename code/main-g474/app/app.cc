#include <stdio.h>

extern "C" {
#include "main.h"
#include "console.h"
#include "usb_app.h"
#include "app/bellow.h"
#include "app/buttons.h"
#include "app/keyboard.h"
#include "app/app.h"
#include "app/midi.h"
#include "app/pedals.h"
#include "app/report.h"

extern UART_HandleTypeDef huart1;
}

static void print_startup_info(void)
{
  extern uint8_t _end;
  extern uint8_t _estack;
  extern uint32_t _Min_Stack_Size;

  const uint32_t ram_base   = 0x20000000UL;
  const uint32_t ram_total  = (uint32_t)&_estack - ram_base;
  const uint32_t stack_resv = (uint32_t)&_Min_Stack_Size;
  const uint32_t static_use = (uint32_t)&_end - ram_base;
  const uint32_t heap_free  = (uint32_t)&_estack - stack_resv - (uint32_t)&_end;

  printf("\r\n=== Bandolibre main ===\r\n");
  printf("Clocks: SYSCLK %lu MHz, HCLK %lu MHz, PCLK1 %lu MHz, PCLK2 %lu MHz\r\n",
         (unsigned long)(HAL_RCC_GetSysClockFreq() / 1000000UL),
         (unsigned long)(HAL_RCC_GetHCLKFreq()     / 1000000UL),
         (unsigned long)(HAL_RCC_GetPCLK1Freq()    / 1000000UL),
         (unsigned long)(HAL_RCC_GetPCLK2Freq()    / 1000000UL));
  printf("USB: %lu MHz kernel clock, %s\r\n",
         (unsigned long)(HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_USB) / 1000000UL),
         usb_app_mounted() ? "enumerated" : "not yet enumerated");
  printf("RAM: %lu KiB total, %lu B static, %lu B stack reserved, %lu B heap free\r\n",
         (unsigned long)(ram_total / 1024UL),
         (unsigned long)static_use,
         (unsigned long)stack_resv,
         (unsigned long)heap_free);
}

void main_task(void)
{
  report_begin();
  usb_app_task();
  midi_poll();
  bellow_poll();
  keyboard_poll();

  static uint32_t last_rate_tick = 0;
  uint32_t rate_now = HAL_GetTick();
  uint32_t rate_dt  = rate_now - last_rate_tick;
  if (rate_dt >= 1000)
  {
    last_rate_tick = rate_now;
    keyboard_print_rates(rate_dt);
  }

  buttons_poll();
  pedals_poll();
  report_end();
  console_task();
}

void main_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
  console_init(&huart1, USART1_IRQn);
  usb_app_init();
  keyboard_init();
  print_startup_info();
}
