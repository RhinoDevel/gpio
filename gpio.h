
// RhinoDevel, Marcel Timm, 2017dec28

#include <stdbool.h>

/** Initialize memory mapped Raspberry PI 2 stuff.
 */
bool gpio_init();

/** Set pin with given nr. to output mode.
 *  Use bool parameter value true for initial high level,
 *  false for initial low level output.
 */
void gpio_set_output(int const pin_nr, bool const high);

/** Set pin with given nr. to input mode with pull up resistor.
 */
void gpio_set_input_pull_up(int const pin_nr);

/** Set pin with given nr. to input mode with pull down resistor.
 */
void gpio_set_input_pull_down(int const pin_nr);

/** Write to pin with given nr.
 *  Pin MUST be set to output mode, before.
 *  Use bool parameter value true for high level, false for low level output.
 */
void gpio_write(int const pin_nr, bool const high);

/** Read from pin with given nr.
 *  Works for pins set as input AND pins set as output, too.
 *  Returns true for high and false for low value read.
 */
bool gpio_read(int const pin_nr);
