#include "debug.h"

#include "stm32g4xx_hal.h"

static UART_HandleTypeDef huart1;

void debug_init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  RCC_PeriphCLKInitTypeDef pclk = {
    .PeriphClockSelection = RCC_PERIPHCLK_USART1,
    .Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2,
  };
  HAL_RCCEx_PeriphCLKConfig(&pclk);

  /* TX only (PA9); RX (PA10) is left alone since nothing here reads input. */
  GPIO_InitTypeDef gpio = {
    .Pin = GPIO_PIN_9,
    .Mode = GPIO_MODE_AF_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_LOW,
    .Alternate = GPIO_AF7_USART1,
  };
  HAL_GPIO_Init(GPIOA, &gpio);

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);
}

/* Routes printf() straight to the UART, blocking. Strong definition — same
 * trick ../common/console/console.c uses for the application — so newlib's
 * weak _write()/__io_putchar() chain in Core/Src/syscalls.c (not built here
 * anyway) never comes into it. */
int _write(int fd, char *buf, int len)
{
  (void) fd;
  HAL_UART_Transmit(&huart1, (uint8_t *) buf, (uint16_t) len, HAL_MAX_DELAY);
  return len;
}
