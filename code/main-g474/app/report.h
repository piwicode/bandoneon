#ifndef APP_REPORT_H
#define APP_REPORT_H

#include <stdbool.h>

/* Live-metrics scheduler. The dashboard is the fixed top block of the console
 * (console_dash_* in console.h); each subsystem renders its section from inside
 * its own *_poll() when a frame is due. This module is the bridge between the
 * property system (timing/enable) and the console: it owns the per-iteration
 * frame bracket so main.c stays terse and console.c stays property-agnostic.
 *
 * Main loop wiring:
 *   report_begin();        // opens a frame if one is due; publishes g_report_due
 *   ...polls render their show_* section while g_report_due...
 *   report_end();          // closes/tears down the frame
 */

/* Set by report_begin() each iteration; read by the polls to decide whether to
 * emit their dashboard section this iteration. */
extern bool g_report_due;

/* Begin a dashboard frame for this iteration: decide whether one is due, publish
 * the result as g_report_due, open the console frame if due, and return the flag. */
bool report_begin(void);

/* End the frame: close the console dashboard (drawing the rule at the current
 * period, report_period_ms) if one was opened this iteration, otherwise tear the dashboard down
 * once no report is enabled. */
void report_end(void);

#endif /* APP_REPORT_H */
