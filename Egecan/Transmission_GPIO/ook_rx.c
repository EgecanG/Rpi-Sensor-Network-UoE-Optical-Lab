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

#define SAMPLES_PER_BIT 10     // Take 10 samples per bit period
#define BIT_DURATION_NS 250    // 250ns bit period
#define SAMPLE_INTERVAL_NS (BIT_DURATION_NS / SAMPLES_PER_BIT)
#define GPIO_PIN 4             // Using GPIO 4

#define MESSAGE_SIZE 8         // 8-byte message size
#define PREAMBLE_BYTES 2       // 2-byte preamble
#define START_SEQUENCE 0xAA    // Start sequence byte
#define END_SEQUENCE 0x55      // End sequence byte

#define STATE_LOOKING_FOR_PREAMBLE 0
#define STATE_CHECKING_START 1
#define STATE_RECEIVING_DATA 2

volatile unsigned *gpio;
uint32_t cycles_per_bit;
uint32_t cycles_per_sample;
uint64_t sample_count = 0;
struct timespec start_time;

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
    cycles_per_sample = cycles_per_bit / SAMPLES_PER_BIT;
    
    printf("CPU Frequency: %lu Hz\n", cpu_freq);
    printf("Cycles per bit: %u\n", cycles_per_bit);
    printf("Cycles per sample: %u\n", cycles_per_sample);
    
    clock_gettime(CLOCK_MONOTONIC, &start_time);
}

inline int read_bit(uint64_t *next_sample_cycle) {
    int high_samples = 0;
    
    // Take samples over one bit period
    for(int i = 0; i < SAMPLES_PER_BIT; i++) {
        while (get_cycles() < *next_sample_cycle) {
            asm volatile("nop");
        }
        
        if (GPIO_GET(GPIO_PIN)) {
            high_samples++;
        }
        
        *next_sample_cycle += cycles_per_sample;
        sample_count++;
    }
    
    // Majority vote for bit value
    return (high_samples > SAMPLES_PER_BIT/2);
}

uint8_t read_byte(uint64_t *next_sample_cycle) {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        if (read_bit(next_sample_cycle)) {
            byte |= (1 << i);
        }
    }
    return byte;
}

int detect_preamble(void) {
    uint64_t next_sample_cycle = get_cycles() + cycles_per_sample;
    uint8_t preamble_bytes[PREAMBLE_BYTES];
    
    for (int i = 0; i < PREAMBLE_BYTES; i++) {
        preamble_bytes[i] = read_byte(&next_sample_cycle);
        if (preamble_bytes[i] != 0xAA) {
            return 0;
        }
    }
    return 1;
}

void receive_frames(void) {
    uint8_t message[MESSAGE_SIZE];
    uint64_t next_sample_cycle;
    int state = STATE_LOOKING_FOR_PREAMBLE;
    
    printf("Waiting for transmission...\n");
    
    while (1) {
        switch (state) {
            case STATE_LOOKING_FOR_PREAMBLE:
                // Wait for any high signal
                while (!GPIO_GET(GPIO_PIN)) {
                    asm volatile("nop");
                }
                
                if (detect_preamble()) {
                    printf("\nPreamble detected\n");
                    next_sample_cycle = get_cycles() + cycles_per_sample;
                    state = STATE_CHECKING_START;
                }
                break;
                
            case STATE_CHECKING_START:
                if (read_byte(&next_sample_cycle) == START_SEQUENCE) {
                    printf("Start sequence found\n");
                    state = STATE_RECEIVING_DATA;
                } else {
                    printf("Invalid start sequence\n");
                    state = STATE_LOOKING_FOR_PREAMBLE;
                }
                break;
                
            case STATE_RECEIVING_DATA:
                printf("Receiving message: ");
                for (int i = 0; i < MESSAGE_SIZE; i++) {
                    message[i] = read_byte(&next_sample_cycle);
                    printf("%02X ", message[i]);
                }
                printf("\n");
                
                uint8_t end_byte = read_byte(&next_sample_cycle);
                if (end_byte == END_SEQUENCE) {
                    printf("Message as text: ");
                    for (int i = 0; i < MESSAGE_SIZE; i++) {
                        if (message[i] >= 32 && message[i] <= 126) {
                            printf("%c", message[i]);
                        } else {
                            printf(".");
                        }
                    }
                    printf("\n\n");
                } else {
                    printf("Invalid end sequence: %02X\n", end_byte);
                }
                
                state = STATE_LOOKING_FOR_PREAMBLE;
                break;
        }
    }
}

int main(void) {
    // Set process priority
    struct sched_param sp = { .sched_priority = sched_get_priority_max(SCHED_FIFO) };
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        perror("Failed to set real-time priority");
    }

    if (mlockall(MCL_CURRENT|MCL_FUTURE) != 0) {
        perror("mlockall failed");
    }

    setup_io();
    INP_GPIO(GPIO_PIN);

    printf("\nOOK Frame Receiver\n");
    printf("----------------\n");
    printf("Listening on GPIO %d\n", GPIO_PIN);
    printf("Bit duration: %d ns\n", BIT_DURATION_NS);
    printf("Samples per bit: %d\n", SAMPLES_PER_BIT);
    printf("Frame format:\n");
    printf("- Preamble: 2 bytes (0xAA 0xAA)\n");
    printf("- Start sequence: 1 byte (0xAA)\n");
    printf("- Message: 8 bytes\n");
    printf("- End sequence: 1 byte (0x55)\n");
    printf("----------------\n");
    printf("Press Ctrl+C to stop.\n\n");

    receive_frames();

    return 0;
}