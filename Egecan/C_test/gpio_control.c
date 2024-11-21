#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#define BCM2711_PERI_BASE 0xFE000000
#define GPIO_BASE (BCM2711_PERI_BASE + 0x200000)
#define PAGE_SIZE 4096
#define BLOCK_SIZE 4096

#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))

volatile unsigned *gpio;

void setup_io()
{
    int mem_fd;
    void *gpio_map;

    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC)) < 0) {
        printf("Failed to open /dev/mem\n");
        exit(-1);
    }

    gpio_map = mmap(
        NULL,
        BLOCK_SIZE,
        PROT_READ|PROT_WRITE,
        MAP_SHARED,
        mem_fd,
        GPIO_BASE
    );

    close(mem_fd);

    if (gpio_map == MAP_FAILED) {
        printf("mmap error %d\n", (int)gpio_map);
        exit(-1);
    }

    gpio = (volatile unsigned *)gpio_map;
}

int main(int argc, char **argv)
{
    // Set up gpio pointer for direct register access
    setup_io();

    // Configure GPIO 17 as output
    INP_GPIO(17);
    OUT_GPIO(17);

    // Get direct pointers to the SET and CLEAR registers for faster access
    volatile unsigned *gpio_set = gpio + 7;    // GPSET0
    volatile unsigned *gpio_clr = gpio + 10;   // GPCLR0
    
    // Pre-calculate the bit mask for GPIO 17
    unsigned int bit = 1 << 17;

    printf("Starting 2MHz signal generation. Press Ctrl+C to stop.\n");

    // Main loop - tight timing loop for maximum frequency
    while(1) {
        // Set GPIO 17 high
        *gpio_set = bit;
        *gpio_set = bit;
        *gpio_set = bit;
        *gpio_set = bit;
        
        // Set GPIO 17 low
        *gpio_clr = bit;
        *gpio_clr = bit;
        *gpio_clr = bit;
        *gpio_clr = bit;
    }

    return 0;
}