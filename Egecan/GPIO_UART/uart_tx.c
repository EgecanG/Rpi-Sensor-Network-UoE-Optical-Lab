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
#include <stdbool.h>

#define BCM2711_PERI_BASE 0xFE000000
#define GPIO_BASE (BCM2711_PERI_BASE + 0x200000)
#define PAGE_SIZE 4096
#define BLOCK_SIZE 4096

#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))

// UART Configuration
#define BAUD_RATE 9600
#define BIT_DURATION_NS (1000000000 / BAUD_RATE)  // Nanoseconds per bit
#define DATA_BITS 8
#define USE_PARITY true
#define PARITY_EVEN true
#define STOP_BITS 1

#define UART_IDLE_LINE 1  // UART idle state is high

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

// Calculate parity bit (even or odd)
int calculate_parity(uint8_t data) {
    int parity = 0;
    for (int i = 0; i < DATA_BITS; i++) {
        if (data & (1 << i)) {
            parity ^= 1;
        }
    }
    return PARITY_EVEN ? parity : !parity;
}

void transmit_uart_byte(uint8_t byte) {
    // Start bit (always 0)
    transmit_bit(0);
    
    // Data bits (LSB first)
    for (int i = 0; i < DATA_BITS; i++) {
        transmit_bit((byte >> i) & 0x01);
    }
    
    // Parity bit (if enabled)
    if (USE_PARITY) {
        transmit_bit(calculate_parity(byte));
    }
    
    // Stop bit(s) (always 1)
    for (int i = 0; i < STOP_BITS; i++) {
        transmit_bit(1);
    }
}

void transmit_uart_frame(const uint8_t *message, size_t length) {
    // Set line to idle state before transmission
    *gpio_set = gpio_bit;
    usleep(BIT_DURATION_NS / 1000);  // Wait one bit duration
    
    // Transmit each byte using UART protocol
    for (size_t i = 0; i < length; i++) {
        transmit_uart_byte(message[i]);
    }
    
    // Return line to idle state
    *gpio_set = gpio_bit;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: %s <message>\n", argv[0]);
        return 1;
    }

    // Set process priority
    struct sched_param sp = { .sched_priority = sched_get_priority_max(SCHED_FIFO) };
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        perror("Failed to set real-time priority");
    }

    if (mlockall(MCL_CURRENT|MCL_FUTURE) != 0) {
        perror("mlockall failed");
    }

    setup_io();
    INP_GPIO(4);
    OUT_GPIO(4);

    const char *input_message = argv[1];
    size_t message_length = strlen(input_message);
    
    printf("Starting UART transmission:\n");
    printf("Message: \"%s\"\n", input_message);
    printf("Configuration:\n");
    printf("- Baud Rate: %d\n", BAUD_RATE);
    printf("- Data Bits: %d\n", DATA_BITS);
    printf("- Parity: %s\n", USE_PARITY ? (PARITY_EVEN ? "Even" : "Odd") : "None");
    printf("- Stop Bits: %d\n", STOP_BITS);
    printf("- Bit Duration: %d ns\n", BIT_DURATION_NS);
    printf("Press Ctrl+C to stop.\n\n");

    // Set initial line state to idle (high)
    *gpio_set = gpio_bit;
    
    while(1) {
        transmit_uart_frame((const uint8_t *)input_message, message_length);
        usleep(100000);  // Add some delay between transmissions (100ms)
    }

    return 0;
}