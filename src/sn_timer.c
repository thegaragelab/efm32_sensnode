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
#define TICKS_PER_SECOND 1000

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
  uint32_t elapsed;
  if(start>end) // Handle wrap around
    elapsed = ((uint32_t)-1) - end + start;
  else
    elapsed = end - start;
  // Convert to appropriate time period
  switch(units) {
    case MILLISECOND:
      break;
    case SECOND:
      elapsed = elapsed / 1000;
      break;
    }
  return elapsed;
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
  return timeElapsed(reference, getTicks(), units)>=duration;
  }
