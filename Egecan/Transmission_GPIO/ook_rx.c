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
#define GPIO_GET(g) (*(gpio+13)&(1<<g))

#define BIT_DURATION_NS 250
#define SAMPLE_OFFSET_NS 125    // Sample in middle of bit
#define BUFFER_SIZE 1024
#define GPIO_PIN 4              // Using GPIO 4

// Debug flags
#define DEBUG_BITS 1            // Print received bits
#define DEBUG_TIMING 1          // Print timing information

#define STATE_LOOKING_FOR_PREAMBLE 0
#define STATE_CHECKING_START 1
#define STATE_RECEIVING_DATA 2

volatile unsigned *gpio;
uint32_t cycles_per_bit;
uint32_t cycles_per_sample_offset;

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
    cycles_per_sample_offset = (cpu_freq * SAMPLE_OFFSET_NS) / 1000000000ULL;
    
    printf("CPU Frequency: %lu Hz\n", cpu_freq);
    printf("Cycles per bit: %u\n", cycles_per_bit);
    printf("Sample offset cycles: %u\n", cycles_per_sample_offset);
}

// Enhanced bit reading with averaging
inline int read_bit(uint64_t *next_sample_cycle) {
    int samples = 0;
    int high_samples = 0;
    const int NUM_SAMPLES = 3;  // Take multiple samples per bit
    
    for(int i = 0; i < NUM_SAMPLES; i++) {
        while (get_cycles() < *next_sample_cycle + (i * cycles_per_bit/NUM_SAMPLES)) {
            asm volatile("nop");
        }
        
        if (GPIO_GET(GPIO_PIN)) {
            high_samples++;
        }
        samples++;
    }
    
    *next_sample_cycle += cycles_per_bit;
    int bit = (high_samples > (samples/2));  // Majority vote
    
    if (DEBUG_BITS) {
        printf("%d", bit);
        fflush(stdout);
    }
    
    return bit;
}

// Enhanced preamble detection with error tolerance
int detect_preamble(uint64_t *next_sample_cycle) {
    int pattern[8] = {1,0,1,0,1,0,1,0};  // Expected pattern
    int matches = 0;
    int errors = 0;
    const int MAX_ERRORS = 1;  // Allow one error in pattern
    
    printf("\nChecking preamble pattern: ");
    for (int i = 0; i < 8; i++) {
        int bit = read_bit(next_sample_cycle);
        if (bit == pattern[i]) {
            matches++;
        } else {
            errors++;
        }
    }
    printf(" (matches: %d, errors: %d)\n", matches, errors);
    
    return (errors <= MAX_ERRORS);
}

// Enhanced start sequence detection
int check_start_sequence(uint64_t *next_sample_cycle) {
    printf("Reading start sequence: ");
    uint8_t start_byte = 0;
    for (int i = 0; i < 8; i++) {
        if (read_bit(next_sample_cycle)) {
            start_byte |= (1 << i);
        }
    }
    printf(" (0x%02X)\n", start_byte);
    return (start_byte == 0x0F);  // 11110000
}

uint8_t read_byte(uint64_t *next_sample_cycle) {
    if (DEBUG_BITS) printf("Reading byte: ");
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        if (read_bit(next_sample_cycle)) {
            byte |= (1 << i);
        }
    }
    if (DEBUG_BITS) printf(" (0x%02X)\n", byte);
    return byte;
}

void receive_messages(void) {
    uint8_t buffer[BUFFER_SIZE];
    int buffer_pos = 0;
    int state = STATE_LOOKING_FOR_PREAMBLE;
    uint64_t next_sample_cycle;
    uint64_t last_transition = 0;
    int consecutive_fails = 0;
    
    printf("Waiting for transmission...\n");
    
    while (1) {
        switch (state) {
            case STATE_LOOKING_FOR_PREAMBLE: {
                // Wait for stable high signal
                int stable_count = 0;
                while (stable_count < 100) {  // Require 100 consistent samples
                    if (GPIO_GET(GPIO_PIN)) {
                        stable_count++;
                    } else {
                        stable_count = 0;
                    }
                    for(volatile int i = 0; i < 100; i++); // Small delay
                }
                
                next_sample_cycle = get_cycles() + cycles_per_sample_offset;
                
                if (detect_preamble(&next_sample_cycle)) {
                    printf("\nValid preamble detected, checking start sequence...\n");
                    state = STATE_CHECKING_START;
                    consecutive_fails = 0;
                } else {
                    consecutive_fails++;
                    if (consecutive_fails > 10) {
                        printf("\nToo many failed preamble attempts, resetting...\n");
                        usleep(1000);  // Add small delay before retrying
                        consecutive_fails = 0;
                    }
                }
                break;
            }
                
            case STATE_CHECKING_START:
                if (check_start_sequence(&next_sample_cycle)) {
                    state = STATE_RECEIVING_DATA;
                    buffer_pos = 0;
                    printf("Valid start sequence found, receiving data...\n");
                } else {
                    printf("Invalid start sequence, restarting...\n");
                    state = STATE_LOOKING_FOR_PREAMBLE;
                }
                break;
                
            case STATE_RECEIVING_DATA: {
                uint8_t byte = read_byte(&next_sample_cycle);
                
                // Check for end sequence (00001111)
                if (byte == 0xF0) {
                    buffer[buffer_pos] = '\0';
                    printf("\nEnd sequence detected!\n");
                    printf("Message received (%d bytes): '%s'\n", buffer_pos, buffer);
                    printf("Message in hex: ");
                    for(int i = 0; i < buffer_pos; i++) {
                        printf("%02X ", buffer[i]);
                    }
                    printf("\n");
                    state = STATE_LOOKING_FOR_PREAMBLE;
                } else if (byte == 0 && buffer_pos == 0) {
                    // Likely noise or misaligned, restart
                    printf("Zero byte at start, restarting...\n");
                    state = STATE_LOOKING_FOR_PREAMBLE;
                } else {
                    buffer[buffer_pos++] = byte;
                    if (buffer_pos >= BUFFER_SIZE - 1) {
                        printf("Buffer full, resetting...\n");
                        state = STATE_LOOKING_FOR_PREAMBLE;
                    }
                }
                break;
            }
        }
    }
}

int main(void) {
    if (setpriority(PRIO_PROCESS, 0, -20) != 0) {
        perror("Failed to set process priority");
    }

    struct sched_param sp = { .sched_priority = sched_get_priority_max(SCHED_FIFO) };
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        perror("Failed to set real-time priority");
    }

    if (mlockall(MCL_CURRENT|MCL_FUTURE) != 0) {
        perror("mlockall failed");
    }

    setup_io();
    INP_GPIO(GPIO_PIN);

    printf("OOK Signal Receiver (Debug Version)\n");
    printf("Listening on GPIO %d\n", GPIO_PIN);
    printf("Bit duration: %d ns\n", BIT_DURATION_NS);
    printf("Sample offset: %d ns\n", SAMPLE_OFFSET_NS);
    printf("Looking for alternating 1-0 preamble pattern\n");
    printf("Press Ctrl+C to stop.\n\n");

    receive_messages();

    return 0;
}