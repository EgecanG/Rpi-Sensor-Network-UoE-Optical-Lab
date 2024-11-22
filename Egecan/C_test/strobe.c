#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

// BCM2711 (Raspberry Pi 4B) physical addresses
#define BCM2711_PERI_BASE       0xFE000000
#define GPIO_BASE               (BCM2711_PERI_BASE + 0x200000) /* GPIO controller */

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))

// GPIO pin operations
#define GPIO_SET *(gpio+7)  // Set bits which are 1, ignore those which are 0
#define GPIO_CLR *(gpio+10) // Clear bits which are 1, ignore those which are 0

// Global variables
volatile unsigned *gpio;

// Function to map GPIO memory
void setup_io()
{
    int mem_fd;
    void *gpio_map;

    // Open /dev/mem
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
        printf("Can't open /dev/mem\nTry running as root!\n");
        exit(-1);
    }

    // mmap GPIO
    gpio_map = mmap(
        NULL,                   // Any address in our space will do
        BLOCK_SIZE,            // Map length
        PROT_READ|PROT_WRITE,  // Enable reading & writing to mapped memory
        MAP_SHARED,            // Shared with other processes
        mem_fd,                // File to map
        GPIO_BASE             // Offset to GPIO peripheral
    );

    close(mem_fd); // No need to keep mem_fd open after mmap

    if (gpio_map == MAP_FAILED) {
        printf("mmap error %d\n", (int)gpio_map); // errno also set!
        exit(-1);
    }

    gpio = (volatile unsigned *)gpio_map; // Always use volatile pointer!
}

int main(int argc, char **argv)
{
    // Set up gpi pointer for direct register access
    setup_io();

    // Set GPIO pin 4 to output
    INP_GPIO(4); // Must use INP_GPIO before using OUT_GPIO
    OUT_GPIO(4);

    printf("Generating square wave on GPIO 4 (Physical Pin 7)...\n");
    printf("Press CTRL+C to exit\n");

    // Generate square wave
    while(1) {
        GPIO_SET = 1<<4;  // Set GPIO 4 high
        GPIO_CLR = 1<<4;  // Set GPIO 4 low
    }

    return 0;
}