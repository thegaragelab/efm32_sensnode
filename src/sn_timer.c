/*--------------------------------------------------------------------------*
* SensNode - Timing
*---------------------------------------------------------------------------*
* 2015-Nov-04 ShaneG
*
* Set up the timing functions (based on the SYSTICK timer).
*--------------------------------------------------------------------------*/
#include <efm32zg222f32.h>
#include "sensnode.h"

// Number of ticks per second
#define TICKS_PER_SECOND 100

// The current system tick count
static volatile uint32_t g_systicks;

/** Interrupt handler for the SysTick timer
 *
 * Simply increments the counter.
 */
void SysTick_Handler(void)  {
  g_systicks++;
  }

/** Initialise the timer subsystem
 */
void initTICK() {
  SysTick_Config(SystemCoreClock / TICKS_PER_SECOND);
  }

//---------------------------------------------------------------------------
// Public API
//---------------------------------------------------------------------------

/** Get the current system tick count
 *
 * Each processor board maintains a count of system ticks since power up, this
 * function returns the current value of that count. The duration of a single
 * tick is processor independent - use the 'timeExpired()' function to determine
 * if a time period has been exceeded or 'timeElapsed()' to determine the
 * amount of time between two tick counts.
 *
 * @return the current system tick count.
 */
uint32_t getTicks() {
  return g_systicks;
  }

/** Calculate the time difference between two tick counts.
 *
 * This function will convert the difference between two tick count values into
 * an actual time period. Tick count wrap around is accounted for. The return
 * value is the number of whole units rounded down.
 *
 * @param start the tick count at the start of the period
 * @param end the tick count at the end of the period
 * @param units the time units to calculate the difference in.
 *
 * @return the amount of elapsed time in whole units.
 */
uint32_t timeElapsed(uint32_t start, uint32_t end, TIMEUNIT units) {
  return 0;
  }

/** Determine if the specified amount of time has expired.
 *
 * This function compares the current tick count with a previously stored
 * start point and determines if the requested amount of time has expired
 * yet.
 *
 * @param reference the starting reference
 * @param duration the time period we are waiting to expire
 * @param units the units the duration is expressed in
 *
 * @return true if the specified duration has expired since the reference
 *              point.
 */
bool timeExpired(uint32_t reference, uint32_t duration, TIMEUNIT units) {
  return false;
  }

/** Delay the invocation of the application loop for a specified duration
 *
 * This function will block until the specified time period has elapsed but
 * background tasks (network operations, battery monitoring, etc) will continue.
 *
 * @param duration the amount of time to delay for
 * @param units the units the duration is specified in
 */
void delay(uint32_t duration, TIMEUNIT units) {
  // TODO: This should be implemented in main().
  }
