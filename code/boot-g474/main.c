/*
 * Bandolibre UF2 bootloader.
 *
 * Reset always lands here. If nothing asks for DFU mode and the application
 * above looks valid, this hands over to it immediately; otherwise it brings up
 * USB as a mass storage device called BANDOLIBRE and waits for a .uf2 file to
 * be copied onto it.
 *
 * Three things ask for DFU mode:
 *   - SW_FN2 (PB4) held down at power-on;
 *   - the console `dfu` command in the application, via BOOT_FLAG_ADDR;
 *   - an application that fails its vector table check, so a board whose
 *     firmware is missing or half-written recovers by itself.
 *
 * See memmap.h for the flash split and README.md for the whole picture.
 */

#include <stdbool.h>
#include <stdio.h>

#include "debug.h"
#include "memmap.h"
#include "msc_disk.h"
#include "stm32g4xx_hal.h"
#include "tusb.h"

/* SW_FN2, the rightmost function button. Active low. */
#define DFU_BUTTON_PORT   GPIOB
#define DFU_BUTTON_PIN    GPIO_PIN_4

/* LED_FN2, above that button. */
#define DFU_LED_PORT      GPIOB
#define DFU_LED_PIN       GPIO_PIN_5

static volatile uint32_t *const boot_flag = (volatile uint32_t *) BOOT_FLAG_ADDR;

void Error_Handler(void);

/* Clock tree, copied from the application's CubeMX SystemClock_Config()
 * (main-g474/Core/Src/main.c). HSI16 through the PLL for a 96 MHz core, and —
 * the part that matters here — HSI48 as the USB kernel clock with the CRS
 * trimming it against USB start-of-frame. The board is crystal-less, so
 * without the CRS the 48 MHz drifts well outside the ±0.25% that full-speed
 * USB allows and enumeration becomes unreliable.
 * code/tests/test_usb_config.py guards the .ioc side of this. */
static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};
  RCC_CRSInitTypeDef crs = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
  osc.HSIState = RCC_HSI_ON;
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc.HSI48State = RCC_HSI48_ON;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  osc.PLL.PLLM = RCC_PLLM_DIV1;
  osc.PLL.PLLN = 12;
  osc.PLL.PLLP = RCC_PLLP_DIV2;
  osc.PLL.PLLQ = RCC_PLLQ_DIV4;
  osc.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV1;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3) != HAL_OK) Error_Handler();

  __HAL_RCC_CRS_CLK_ENABLE();

  crs.Prescaler = RCC_CRS_SYNC_DIV1;
  crs.Source = RCC_CRS_SYNC_SOURCE_USB;
  crs.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  crs.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
  crs.ErrorLimitValue = 34;
  crs.HSI48CalibrationValue = 32;
  HAL_RCCEx_CRSConfig(&crs);
}

/* Called by HAL_Init(). Mirrors the application's generated HAL_MspInit()
 * (main-g474/Core/Src/stm32g4xx_hal_msp.c).
 *
 * The dead-battery call is not boilerplate here — it is what makes the DFU
 * button readable at all. PB4 and PB6 are UCPD1_CC2 and UCPD1_CC1, and out of
 * reset each can carry a 5.1 kOhm pull-down from the UCPD peripheral, gated by
 * the level on PA10 and PA9 respectively (STM32G474CB datasheet, pin table
 * note 6). PA10 is this board's USART1_RX, which the attached ST-Link VCP
 * holds high while idle — so PB4's pull-down is active, it beats the ~40 kOhm
 * internal pull-up, and SW_FN2 would read as pressed forever. Setting
 * UCPD1_DBDIS in PWR_CR3 removes it. */
void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  HAL_PWREx_DisableUCPDDeadBattery();
}

static void gpio_init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* PB4 is also NJTRST and comes out of reset in its JTAG alternate function
   * (datasheet note 5), so it has to be claimed as a plain input. The board is
   * debugged over SWD, which does not use this pin. */
  GPIO_InitTypeDef in = {
    .Pin = DFU_BUTTON_PIN,
    .Mode = GPIO_MODE_INPUT,
    .Pull = GPIO_PULLUP,
  };
  HAL_GPIO_Init(DFU_BUTTON_PORT, &in);

  GPIO_InitTypeDef out = {
    .Pin = DFU_LED_PIN,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_LOW,
  };
  HAL_GPIO_Init(DFU_LED_PORT, &out);
  HAL_GPIO_WritePin(DFU_LED_PORT, DFU_LED_PIN, GPIO_PIN_RESET);
}

static bool dfu_button_pressed(void)
{
  /* Let the pull-up charge the pin and the button's stray capacitance before
   * sampling; this runs microseconds after the pin was reconfigured. */
  HAL_Delay(2);
  return HAL_GPIO_ReadPin(DFU_BUTTON_PORT, DFU_BUTTON_PIN) == GPIO_PIN_RESET;
}

/* Cheap sanity check on the application's vector table: a plausible initial
 * stack pointer and a reset handler inside the application region. This is
 * what catches a missing or half-written image — flash_write_app() programs
 * the vector table page last precisely so that an interrupted update fails
 * here rather than jumping into nothing. */
static bool app_valid(void)
{
  uint32_t const sp = ((uint32_t const *) APP_BASE)[0];
  uint32_t const pc = ((uint32_t const *) APP_BASE)[1];

  if (sp < 0x20000000UL || sp > 0x20020000UL) return false;
  if (pc < APP_BASE || pc >= APP_BASE + APP_SIZE) return false;
  if ((pc & 1UL) == 0UL) return false;   /* Thumb bit */

  return true;
}

static void jump_to_app(void)
{
  uint32_t const sp = ((uint32_t const *) APP_BASE)[0];
  uint32_t const pc = ((uint32_t const *) APP_BASE)[1];

  /* Hand the application the same machine it would see after a cold reset:
   * clocks back on HSI, no peripheral left running, no interrupt pending.
   *
   * This runs with interrupts still enabled on purpose. The HAL's RCC teardown
   * polls HAL_GetTick() for its timeouts, and the tick only advances from the
   * SysTick interrupt — with interrupts off, a clock that failed to come ready
   * would spin here forever instead of returning HAL_TIMEOUT. Nothing else is
   * running to be disturbed: this path is only reached when DFU mode was not
   * entered, so USB was never initialised and SysTick is the only live
   * interrupt. */
  HAL_RCC_DeInit();
  HAL_DeInit();

  __disable_irq();

  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL  = 0;

  for (uint32_t i = 0; i < 8; i++) {
    NVIC->ICER[i] = 0xFFFFFFFFUL;
    NVIC->ICPR[i] = 0xFFFFFFFFUL;
  }

  SCB->VTOR = APP_BASE;
  __DSB();
  __ISB();

  __set_MSP(sp);
  __set_CONTROL(0);
  __ISB();

  /* PRIMASK is not cleared by reset, and nothing in the application's startup
   * path clears it either, so re-enable interrupts before handing over. */
  __enable_irq();

  ((void (*)(void)) pc)();

  while (1) { }   /* not reached */
}

static void usb_init(void)
{
  /* TinyUSB's dcd_init() does not touch the RCC; in the application this is
   * done by the CubeMX-generated HAL_PCD_MspInit(). There is no PCD here. */
  __HAL_RCC_USB_CLK_ENABLE();

  NVIC_SetPriority(USB_HP_IRQn, 6);
  NVIC_SetPriority(USB_LP_IRQn, 6);
  NVIC_SetPriority(USBWakeUp_IRQn, 6);

  tusb_rhport_init_t dev_init = {
    .role = TUSB_ROLE_DEVICE,
    .speed = TUSB_SPEED_FULL,
  };
  tusb_init(0, &dev_init);
}

/* Slow blink while waiting, fast blink while blocks are arriving. */
static void led_task(void)
{
  uint32_t const period = uf2_receiving() ? 100U : 500U;
  HAL_GPIO_WritePin(DFU_LED_PORT, DFU_LED_PIN,
                    ((HAL_GetTick() / period) & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void dfu_mode(void)
{
  usb_init();

  while (1) {
    tud_task();
    led_task();

    if (uf2_commit_ready()) {
      /* Keep servicing USB briefly so the transfer that completed the image
       * gets its status stage and the host is not left waiting mid-command;
       * programming blocks for about a second. */
      uint32_t const settle = HAL_GetTick();
      while (HAL_GetTick() - settle < 100U) tud_task();

      HAL_GPIO_WritePin(DFU_LED_PORT, DFU_LED_PIN, GPIO_PIN_SET);

      printf("dfu: all blocks received, committing to flash...\r\n");
      if (uf2_commit()) {
        printf("dfu: commit OK, resetting\r\n");
        NVIC_SystemReset();
      }

      /* Programming failed. The application region is now partly erased, so
       * app_valid() will keep the board here after a reset — stay put and let
       * the user retry the copy rather than rebooting into nothing. */
      printf("dfu: commit FAILED, staying in DFU mode\r\n");
      HAL_GPIO_WritePin(DFU_LED_PORT, DFU_LED_PIN, GPIO_PIN_RESET);
    }
  }
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  gpio_init();
  debug_init();

  bool const soft_request = (*boot_flag == BOOT_FLAG_MAGIC);
  *boot_flag = 0;   /* one shot: never trap the board in DFU mode */
  bool const button = dfu_button_pressed();
  bool const valid = app_valid();

  printf("\r\nboot-g474: soft_request=%d button=%d app_valid=%d\r\n",
         soft_request, button, valid);

  if (soft_request || button || !valid) {
    printf("boot-g474: entering DFU mode\r\n");
    dfu_mode();
  }

  printf("boot-g474: jumping to application at 0x%08lx\r\n", (unsigned long) APP_BASE);
  jump_to_app();
  return 0;
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

/* HAL_Delay() and HAL_GetTick() need the tick interrupt. In the application
 * this handler comes from the CubeMX-generated stm32g4xx_it.c, which this
 * project does not use; without it the weak symbol in the startup file points
 * at Default_Handler and the first HAL_Delay() would hang. */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/* USB interrupt handlers (override the weak defaults from the startup file) */
void USB_HP_IRQHandler(void)
{
  tud_int_handler(0);
}

void USB_LP_IRQHandler(void)
{
  tud_int_handler(0);
}

void USBWakeUp_IRQHandler(void)
{
  tud_int_handler(0);
}
