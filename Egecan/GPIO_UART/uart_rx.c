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
#include <signal.h>

#define BCM2711_PERI_BASE 0xFE000000
#define GPIO_BASE (BCM2711_PERI_BASE + 0x200000)
#define PAGE_SIZE 4096
#define BLOCK_SIZE 4096

#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define GPIO_GET(g) (*(gpio+13)&(1<<(g))) // 0 if low, >0 if high

// UART Configuration - must match transmitter
#define BAUD_RATE 2000000
#define BIT_DURATION_NS (1000000000 / BAUD_RATE)  // Nanoseconds per bit
#define DATA_BITS 8
#define USE_PARITY true
#define PARITY_EVEN true
#define STOP_BITS 1

#define SAMPLE_POINT 0.5  // Sample at 50% of bit period
#define MAX_MESSAGE_SIZE 1024
#define GPIO_PIN 4  // Using GPIO 4 for receiving

volatile unsigned *gpio;
uint32_t cycles_per_bit;
bool running = true;

// Buffer for received message
char received_message[MAX_MESSAGE_SIZE];
size_t message_length = 0;

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
    
    uint64_t cpu_freq = get_cpu_freq();
    cycles_per_bit = (cpu_freq * BIT_DURATION_NS) / 1000000000ULL;
    
    printf("CPU Frequency: %lu Hz\n", cpu_freq);
    printf("Cycles per bit: %u\n", cycles_per_bit);
}

// Read the current state of the GPIO pin
inline bool read_gpio(void) {
    return GPIO_GET(GPIO_PIN) != 0;
}

// Wait for the start bit (falling edge)
bool wait_for_start_bit(void) {
    // Wait for line to be idle (high)
    while (!read_gpio() && running) {
        asm volatile("nop");
    }
    
    // Wait for falling edge (start bit)
    while (read_gpio() && running) {
        asm volatile("nop");
    }
    
    if (!running) return false;
    
    // Move to the center of the start bit
    uint64_t start_cycle = get_cycles();
    while ((get_cycles() - start_cycle) < (cycles_per_bit / 2)) {
        asm volatile("nop");
    }
    
    // Verify we're still in the start bit
    if (read_gpio()) {
        return false; // False start
    }
    
    return true;
}

// Calculate parity for verification
int calculate_parity(uint8_t data) {
    int parity = 0;
    for (int i = 0; i < DATA_BITS; i++) {
        if (data & (1 << i)) {
            parity ^= 1;
        }
    }
    return PARITY_EVEN ? parity : !parity;
}

// Receive one UART byte
bool receive_uart_byte(uint8_t *byte) {
    uint8_t data = 0;
    int parity_received = 0;
    uint64_t bit_start = get_cycles();
    
    // Read data bits
    for (int i = 0; i < DATA_BITS; i++) {
        // Wait until middle of bit
        while ((get_cycles() - bit_start) < cycles_per_bit) {
            asm volatile("nop");
        }
        
        // Sample the bit
        if (read_gpio()) {
            data |= (1 << i);
        }
        
        bit_start += cycles_per_bit;
    }
    
    // Read parity bit if enabled
    if (USE_PARITY) {
        while ((get_cycles() - bit_start) < cycles_per_bit) {
            asm volatile("nop");
        }
        parity_received = read_gpio();
        bit_start += cycles_per_bit;
        
        // Verify parity
        if (parity_received != calculate_parity(data)) {
            return false;
        }
    }
    
    // Verify stop bit(s)
    for (int i = 0; i < STOP_BITS; i++) {
        while ((get_cycles() - bit_start) < cycles_per_bit) {
            asm volatile("nop");
        }
        if (!read_gpio()) {
            return false; // Invalid stop bit
        }
        bit_start += cycles_per_bit;
    }
    
    *byte = data;
    return true;
}

void signal_handler(int signum) {
    running = false;
}

int main(void) {
    // Set up signal handler for clean exit
    signal(SIGINT, signal_handler);
    
    // Set process priority
    struct sched_param sp = { .sched_priority = sched_get_priority_max(SCHED_FIFO) };
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        perror("Failed to set real-time priority");
    }

    if (mlockall(MCL_CURRENT|MCL_FUTURE) != 0) {
        perror("mlockall failed");
    }

    setup_io();
    INP_GPIO(GPIO_PIN);  // Set GPIO as input

    printf("Starting UART receiver:\n");
    printf("Configuration:\n");
    printf("- Baud Rate: %d\n", BAUD_RATE);
    printf("- Data Bits: %d\n", DATA_BITS);
    printf("- Parity: %s\n", USE_PARITY ? (PARITY_EVEN ? "Even" : "Odd") : "None");
    printf("- Stop Bits: %d\n", STOP_BITS);
    printf("- GPIO Pin: %d\n", GPIO_PIN);
    printf("Waiting for data...\n\n");
    
    while (running) {
        uint8_t received_byte;
        message_length = 0;
        
        // Wait for and receive data
        while (running && message_length < MAX_MESSAGE_SIZE - 1) {
            if (!wait_for_start_bit()) {
                continue;
            }
            
            if (receive_uart_byte(&received_byte)) {
                received_message[message_length++] = received_byte;
                
                // If we receive a null terminator or newline, print the message
                if (received_byte == '\0' || received_byte == '\n') {
                    received_message[message_length] = '\0';
                    printf("Received message: %s\n", received_message);
                    message_length = 0;
                    break;
                }
            }
        }
        
        // If we accumulated data but didn't get a terminator, print it anyway
        if (message_length > 0) {
            received_message[message_length] = '\0';
            printf("Received message: %s\n", received_message);
        }
    }
    
    printf("\nReceiver stopped.\n");
    return 0;
}