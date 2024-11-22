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
#include <signal.h>

#define BCM2711_PERI_BASE 0xFE000000
#define GPIO_BASE (BCM2711_PERI_BASE + 0x200000)
#define PAGE_SIZE 4096
#define BLOCK_SIZE 4096

#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))

// UART Configuration
#define BAUD_RATE 100000
#define BIT_DURATION_NS (1000000000 / BAUD_RATE)
#define DATA_BITS 8
#define USE_PARITY false
#define STOP_BITS 1

// Test Pattern Configuration
#define TEST_PATTERN_SIZE 64
#define PATTERN_REPEAT_COUNT 100  // Number of times to repeat pattern before stats

volatile unsigned *gpio;
volatile unsigned *gpio_set;
volatile unsigned *gpio_clr;
unsigned int gpio_bit;
uint32_t cycles_per_bit;
bool running = true;

// Test pattern definition
const uint8_t test_pattern[TEST_PATTERN_SIZE] = {
    0x55, 0xAA, 0x00, 0xFF, 0x0F, 0xF0, 0x33, 0xCC,  // Alternating patterns
    0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0,  // Counting pattern
    'H', 'e', 'l', 'l', 'o', ' ', 'W', 'o',          // ASCII text
    'r', 'l', 'd', '!', 0x0D, 0x0A, 0x00, 0xFF,      // Mixed pattern
    0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,  // Single bit shifts
    0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01,  // Reverse bit shifts
    0x1F, 0x2F, 0x3F, 0x4F, 0x5F, 0x6F, 0x7F, 0x8F,  // Mixed bits
    0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8   // More mixed bits
};

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
    for (int i = 0; i < DATA_BITS; i++) {
        transmit_bit((byte >> i) & 0x01);
    }
    
    // Stop bit(s) (always 1)
    for (int i = 0; i < STOP_BITS; i++) {
        transmit_bit(1);
    }
}

void transmit_pattern(void) {
    // Small delay between pattern transmissions
    *gpio_set = gpio_bit;  // Set line idle
    usleep(1000);  // 1ms gap between patterns
    
    // Transmit each byte of the pattern
    for (int i = 0; i < TEST_PATTERN_SIZE; i++) {
        transmit_byte(test_pattern[i]);
    }
}

void signal_handler(int signum) {
    running = false;
}

int main(void) {
    signal(SIGINT, signal_handler);
    
    // Set CPU affinity to core 3
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(3, &set);
    if (sched_setaffinity(0, sizeof(set), &set) == -1) {
        perror("Failed to set CPU affinity");
    }
    
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
    
    printf("Starting test pattern transmission:\n");
    printf("- Pattern size: %d bytes\n", TEST_PATTERN_SIZE);
    printf("- Baud Rate: %d\n", BAUD_RATE);
    printf("- Data Bits: %d\n", DATA_BITS);
    printf("- Stop Bits: %d\n", STOP_BITS);
    printf("Press Ctrl+C to stop.\n\n");

    // Print test pattern for verification
    printf("Test Pattern (hex):\n");
    for (int i = 0; i < TEST_PATTERN_SIZE; i++) {
        printf("%02X ", test_pattern[i]);
        if ((i + 1) % 16 == 0) printf("\n");
    }
    printf("\n");

    unsigned long patterns_sent = 0;
    while(running) {
        transmit_pattern();
        patterns_sent++;
        
        if (patterns_sent % PATTERN_REPEAT_COUNT == 0) {
            printf("Sent %lu test patterns\n", patterns_sent);
        }
    }

    printf("\nTransmission stopped. Total patterns sent: %lu\n", patterns_sent);
    return 0;
}