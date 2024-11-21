#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>

#define BCM2711_PERI_BASE 0xFE000000
#define GPIO_BASE (BCM2711_PERI_BASE + 0x200000)
#define PAGE_SIZE 4096
#define BLOCK_SIZE 4096

#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))

volatile unsigned *gpio;

// Function prototype for delay_ns
void delay_ns(unsigned int ns);

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
        printf("mmap error\n");
        exit(-1);
    }

    gpio = (volatile unsigned *)gpio_map;
}

// Implementation of delay_ns function
void delay_ns(unsigned int ns) {
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC_RAW, &start);
    
    do {
        clock_gettime(CLOCK_MONOTONIC_RAW, &end);
        uint64_t delta_ns = (end.tv_sec - start.tv_sec) * 1000000000ULL + 
                           (end.tv_nsec - start.tv_nsec);
        if (delta_ns >= ns)
            break;
    } while (1);
}

void generate_frequency(unsigned int delay)
{
    // Get direct pointers to the SET and CLEAR registers
    volatile unsigned *gpio_set = gpio + 7;    // GPSET0
    volatile unsigned *gpio_clr = gpio + 10;   // GPCLR0
    
    // Pre-calculate the bit mask for GPIO 17
    unsigned int bit = 1 << 17;

    float freq = 1000000000.0f / (delay * 2); // Calculate frequency in Hz
    printf("Attempting to generate %.2f Hz signal\n", freq);
    printf("Delay time: %u nanoseconds\n", delay);
    printf("Press Ctrl+C to stop.\n");

    // Main loop
    while(1) {
        // Set GPIO 17 high
        *gpio_set = bit;
        delay_ns(delay);
        
        // Set GPIO 17 low
        *gpio_clr = bit;
        delay_ns(delay);
    }
}

int main(int argc, char **argv)
{
    // Set up gpio pointer for direct register access
    setup_io();

    // Configure GPIO 17 as output
    INP_GPIO(17);
    OUT_GPIO(17);

    // Example: for 5MHz (200ns period, so 100ns delay)
    generate_frequency(10);  // 100ns delay for each half-cycle

    return 0;
}