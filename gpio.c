
// RhinoDevel, Marcel Timm, 2017dec28

#include "gpio.h"

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <assert.h>

// Physical addresses
#define BCM2835_PERI_BASE 0x20000000 // Raspi 1
#define BCM2836_PERI_BASE 0x3F000000 // Raspi 2

#define PERI_BASE BCM2836_PERI_BASE

#define GPIO_BASE (PERI_BASE + 0x200000)
#define BLOCK_SIZE (4*1024)

static int const mode_input = 0;
static int const mode_output = 1;

static int const pull_down = 1;
static int const pull_up = 2;

#define GPFSEL ((volatile unsigned int *)(gpio+0))
#define GPSET ((volatile unsigned int *)(gpio+7))
#define GPCLR ((volatile unsigned int *)(gpio+10))
#define GPLEV ((volatile unsigned int *)(gpio+13))
#define GPPULL ((volatile unsigned int *)(gpio+37)) // Pull up / pull down.
#define GPPULLCLK0 ((volatile unsigned int *)(gpio+38)) // Pull up / pull down clock.

#define PIN_OFFSET(n) (n<32 ? 0 : 1) // Register offset.
#define PIN_VAL(n) (1<<(n%32))

static volatile unsigned int * gpio = NULL; // See gpio_init().

static void short_wait()
{
    usleep(250000); // 250ms.
}

static void set_pin_mode(int const pin_nr, int const mode)
{
    assert(gpio!=NULL);
    assert(mode==mode_input || mode==mode_output);

    unsigned int const offset = pin_nr/10, // FPSEL register offset.
        shift = (pin_nr%10)*3; // Bit shift.

    GPFSEL[offset] &= ~(7<<shift); // Clears bits.

    GPFSEL[offset] |= mode<<shift; // Set mode/function.
}

static void set_input_resistor_pull(int const pin_nr, int const pull)
{
    assert(gpio!=NULL);
    assert(pull==pull_down || pull==pull_up);

    *GPPULL = pull;
    short_wait();

    *GPPULLCLK0 = 1<<pin_nr; // Clock on GPIO pin (bit set).
    short_wait();

    *GPPULL = 0;
    *GPPULLCLK0 = 0;
}

static void set_input_pull(int const pin_nr, int const pull)
{
    set_pin_mode(pin_nr, mode_input);
    set_input_resistor_pull(pin_nr, pull);
}

bool gpio_init()
{
    int mem_dev = -1;
    void* mem_map = NULL;

    if(gpio!=NULL)
    {
        printf("gpio_init : Error: Seems to be already initialized!");
        return false;
    }

    mem_dev = open("/dev/mem", O_RDWR|O_SYNC);
    if(mem_dev==-1)
    {
        printf("gpio_init : Error: Can't open \"/dev/mem\"!\n");
        return false;
    }

    mem_map = mmap(
        NULL, // Local mapping start address (NULL means don't care).
        BLOCK_SIZE, // Mapped memory block size.
        PROT_READ|PROT_WRITE, // Enable read and write.
        MAP_SHARED, // No exclusive access.
        mem_dev,
        GPIO_BASE); // Offset to GPIO peripheral.

    close(mem_dev);
    mem_dev = -1;

    if(mem_map==MAP_FAILED)
    {
        printf("gpio_init : Error: Failed to create memory mapping!\n");
        return false;
    }

    gpio = (volatile unsigned *)mem_map;
    return true;

    // [munmap() call is not necessary later,
    // because it will automatically unmap on process termination]
}

void gpio_set_input_pull_up(int const pin_nr)
{
    set_input_pull(pin_nr, pull_up);
}

void gpio_set_input_pull_down(int const pin_nr)
{
    set_input_pull(pin_nr, pull_down);
}

void gpio_write(int const pin_nr, bool const high)
{
    assert(gpio!=NULL);

    if(high)
    {
        GPSET[PIN_OFFSET(pin_nr)] = PIN_VAL(pin_nr);
        return;
    }
    GPCLR[PIN_OFFSET(pin_nr)] = PIN_VAL(pin_nr);
}

bool gpio_read(int const pin_nr)
{
    assert(gpio!=NULL);

    return GPLEV[PIN_OFFSET(pin_nr)] & PIN_VAL(pin_nr);
}

void gpio_set_output(int const pin_nr, bool const high)
{
    set_pin_mode(pin_nr, mode_output);
    gpio_write(pin_nr, high);
}
