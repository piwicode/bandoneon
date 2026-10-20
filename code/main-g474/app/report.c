#include "report.h"

#include "console.h"     /* console_dash_* */
#include "main.h"        /* HAL_GetTick */
#include "properties.h"

bool g_report_due = false;

/* True when any show_* report is enabled (the dashboard should be on screen). */
static bool report_active(void)
{
  return g_properties->show_bellow || g_properties->show_spi ||
         g_properties->show_keyboard;
}

/* True at most once per refresh period (report_period_ms) and only while
 * report_active(): the iterations on which the dashboard is rebuilt. */
static bool report_due(void)
{
  static uint32_t last_tick;
  static bool have_last;

  if (!report_active()) { have_last = false; return false; }

  uint32_t now = HAL_GetTick();
  uint32_t period = g_properties->report_period_ms;
  if (have_last && (now - last_tick) < period) return false;

  last_tick = now;
  have_last = true;
  return true;
}

bool report_begin(void)
{
  g_report_due = report_due();
  if (g_report_due) console_dash_begin();
  return g_report_due;
}

void report_end(void)
{
  if (g_report_due) console_dash_end(g_properties->report_period_ms);
  else if (!report_active()) console_dash_hide();
}
