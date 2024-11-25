#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include <string.h>
#include <sched.h>
#include <sys/time.h>
#include <sys/resource.h>

#define BCM2711_PERI_BASE 0xFE000000
#define GPIO_BASE (BCM2711_PERI_BASE + 0x200000)
#define PAGE_SIZE 4096
#define BLOCK_SIZE 4096

#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))

#define BAUD_RATE 9600
#define BIT_DURATION_NS (1000000000 / BAUD_RATE)

volatile unsigned *gpio;
volatile unsigned *gpio_set;
volatile unsigned *gpio_clr;
unsigned int gpio_bit;
uint32_t cycles_per_bit;

static inline uint64_t get_cycles(void) {
    uint64_t cycles;
    asm volatile("mrs %0, cntvct_el0" : "=r" (cycles));
    return cycles;
}

uint64_t get_cpu_freq(void) {
    uint64_t freq;
    asm volatile("mrs %0, cntfrq_el0" : "=r" (freq));
    return freq;
}

void setup_io(void) {
    int mem_fd;
    void *gpio_map;

    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC)) < 0) {
        printf("Failed to open /dev/mem\n");
        exit(-1);
    }

    gpio_map = mmap(NULL, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, GPIO_BASE);
    close(mem_fd);

    if (gpio_map == MAP_FAILED) {
        printf("mmap error\n");
        exit(-1);
    }

    gpio = (volatile unsigned *)gpio_map;
    gpio_set = gpio + 7;
    gpio_clr = gpio + 10;
    gpio_bit = 1 << 4;  // GPIO 4

    uint64_t cpu_freq = get_cpu_freq();
    cycles_per_bit = (cpu_freq * BIT_DURATION_NS) / 1000000000ULL;
    
    printf("CPU Frequency: %lu Hz\n", cpu_freq);
    printf("Cycles per bit: %u\n", cycles_per_bit);
}

inline void transmit_bit(int bit) {
    uint64_t start_cycle = get_cycles();
    
    if (bit) {
        *gpio_set = gpio_bit;
    } else {
        *gpio_clr = gpio_bit;
    }
    
    while ((get_cycles() - start_cycle) < cycles_per_bit) {
        asm volatile("nop");
    }
}

void transmit_byte(uint8_t byte) {
    // Start bit (always 0)
    transmit_bit(0);
    
    // Data bits (LSB first)
    for (int i = 0; i < 8; i++) {
        transmit_bit((byte >> i) & 0x01);
    }
    
    // Stop bit (always 1)
    transmit_bit(1);
}

void transmit_string(const char *str) {
    while (*str) {
        transmit_byte(*str++);
    }
    // Add newline
    transmit_byte('\r');
    transmit_byte('\n');
}

int main(void) {
    struct sched_param sp = { .sched_priority = sched_get_priority_max(SCHED_FIFO) };
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        perror("Failed to set real-time priority");
    }

    setup_io();
    INP_GPIO(4);
    OUT_GPIO(4);

    const char *test_message = "Hello World! This is a test message.";
    printf("Starting transmission of: %s\n", test_message);
    printf("Baud Rate: %d\n", BAUD_RATE);
    printf("Press Ctrl+C to stop.\n\n");

    *gpio_set = gpio_bit;  // Set line to idle state (high)
    usleep(100000);  // Wait 100ms before starting

    while(1) {
        transmit_string(test_message);
        usleep(100000);  // 100ms delay between transmissions
    }

    return 0;
}