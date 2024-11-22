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

#define SAMPLES_PER_BIT 4      // Take 4 samples per bit period
#define BIT_DURATION_NS 250    // 250ns bit period = 4MHz
#define SAMPLE_INTERVAL_NS (BIT_DURATION_NS / SAMPLES_PER_BIT)
#define GPIO_PIN 4             // Using GPIO 4
#define BUFFER_SIZE 1024

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

uint64_t get_time_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

void set_process_priority(void) {
    // Set maximum priority
    if (setpriority(PRIO_PROCESS, 0, -20) != 0) {
        perror("Failed to set process priority");
    }

    // Set real-time FIFO scheduler
    struct sched_param sp = { .sched_priority = sched_get_priority_max(SCHED_FIFO) };
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        perror("Failed to set real-time priority");
    }
}

void setup_io(void) {
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

    uint64_t cpu_freq = get_cpu_freq();
    cycles_per_bit = (cpu_freq * BIT_DURATION_NS) / 1000000000ULL;
    cycles_per_sample = cycles_per_bit / SAMPLES_PER_BIT;
    
    printf("CPU Frequency: %lu Hz\n", cpu_freq);
    printf("Cycles per bit: %u\n", cycles_per_bit);
    printf("Cycles per sample: %u\n", cycles_per_sample);
    printf("Target sampling interval: %d ns\n", SAMPLE_INTERVAL_NS);
    printf("Target sampling rate: %d MHz\n", 1000 / SAMPLE_INTERVAL_NS);
    
    clock_gettime(CLOCK_MONOTONIC, &start_time);
}

// Wait for a signal transition
int wait_for_edge(int current_level) {
    int count = 0;
    const int timeout = 1000000;  // Timeout counter
    
    while (count < timeout) {
        if (GPIO_GET(GPIO_PIN) != current_level) {
            return 1;  // Edge detected
        }
        count++;
        asm volatile("nop");
    }
    return 0;  // Timeout
}

// Find the start of a bit by looking for transitions
uint64_t synchronize_bits(void) {
    int transitions = 0;
    uint64_t total_cycles = 0;
    const int REQUIRED_TRANSITIONS = 4;
    
    int current_level = GPIO_GET(GPIO_PIN);
    printf("Initial level: %d\n", current_level);
    
    for (int i = 0; i < REQUIRED_TRANSITIONS; i++) {
        uint64_t start = get_cycles();
        
        if (!wait_for_edge(current_level)) {
            printf("Timeout waiting for edge %d\n", i);
            return 0;
        }
        
        uint64_t end = get_cycles();
        total_cycles += (end - start);
        current_level = !current_level;
        transitions++;
        
        printf("Edge %d detected after %lu cycles\n", i, end - start);
    }
    
    if (transitions == REQUIRED_TRANSITIONS) {
        uint64_t avg_cycles = total_cycles / transitions;
        printf("Average cycles between transitions: %lu\n", avg_cycles);
        return avg_cycles;
    }
    
    return 0;
}

inline int read_bit(uint64_t *next_sample_cycle) {
    int high_samples = 0;
    
    // Take multiple samples per bit period
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
    
    return (high_samples > SAMPLES_PER_BIT/2);
}

void print_sampling_stats(void) {
    struct timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    
    uint64_t elapsed_ns = (current_time.tv_sec - start_time.tv_sec) * 1000000000ULL +
                         (current_time.tv_nsec - start_time.tv_nsec);
    
    double actual_sampling_rate = (double)sample_count / (elapsed_ns / 1000000000.0);
    
    printf("\nSampling Statistics:\n");
    printf("Total samples: %lu\n", sample_count);
    printf("Elapsed time: %.3f seconds\n", elapsed_ns / 1000000000.0);
    printf("Actual sampling rate: %.2f MHz\n", actual_sampling_rate / 1000000.0);
    printf("Samples per bit: %d\n\n", SAMPLES_PER_BIT);
}

int detect_preamble(uint64_t *next_sample_cycle) {
    uint64_t bit_time = synchronize_bits();
    if (bit_time == 0) {
        return 0;
    }
    
    cycles_per_bit = bit_time * 2;
    cycles_per_sample = cycles_per_bit / SAMPLES_PER_BIT;
    *next_sample_cycle = get_cycles() + cycles_per_sample;
    
    printf("Adjusted timing:\n");
    printf("Cycles per bit: %u\n", cycles_per_bit);
    printf("Cycles per sample: %u\n", cycles_per_sample);
    printf("Estimated actual bit duration: %lu ns\n", 
           (bit_time * 2 * 1000000000ULL) / get_cpu_freq());
    
    int bits[8];
    int pattern[8] = {1,0,1,0,1,0,1,0};
    int errors = 0;
    
    printf("Reading preamble bits: [");
    for (int i = 0; i < 8; i++) {
        bits[i] = read_bit(next_sample_cycle);
        printf("%d", bits[i]);
        if (i < 7) printf(",");
        
        if (bits[i] != pattern[i]) {
            errors++;
        }
    }
    printf("] (errors: %d)\n", errors);
    
    print_sampling_stats();
    
    return (errors <= 1);
}

int check_start_sequence(uint64_t *next_sample_cycle) {
    uint8_t start_byte = 0;
    printf("Start sequence bits: [");
    for (int i = 0; i < 8; i++) {
        int bit = read_bit(next_sample_cycle);
        printf("%d", bit);
        if (i < 7) printf(",");
        if (bit) {
            start_byte |= (1 << i);
        }
    }
    printf("]\n");
    printf("Start byte: 0x%02X (expected 0x0F)\n", start_byte);
    return (start_byte == 0x0F);
}

uint8_t read_byte(uint64_t *next_sample_cycle) {
    uint8_t byte = 0;
    printf("Reading byte bits: [");
    for (int i = 0; i < 8; i++) {
        int bit = read_bit(next_sample_cycle);
        printf("%d", bit);
        if (i < 7) printf(",");
        if (bit) {
            byte |= (1 << i);
        }
    }
    printf("] = 0x%02X\n", byte);
    return byte;
}

void receive_messages(void) {
    uint8_t buffer[BUFFER_SIZE];
    int buffer_pos = 0;
    int state = STATE_LOOKING_FOR_PREAMBLE;
    uint64_t next_sample_cycle;
    
    printf("Waiting for transmission...\n");
    
    while (1) {
        switch (state) {
            case STATE_LOOKING_FOR_PREAMBLE:
                // Wait for any high signal
                while (!GPIO_GET(GPIO_PIN)) {
                    asm volatile("nop");
                }
                
                if (detect_preamble(&next_sample_cycle)) {
                    printf("\nValid preamble detected, checking start sequence...\n");
                    state = STATE_CHECKING_START;
                } else {
                    usleep(1000);  // Wait a bit before trying again
                }
                break;
                
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
                
            case STATE_RECEIVING_DATA:
                uint8_t byte = read_byte(&next_sample_cycle);
                
                if (byte == 0xF0) {  // End sequence
                    buffer[buffer_pos] = '\0';
                    printf("\nMessage received (%d bytes): '%s'\n", buffer_pos, buffer);
                    printf("Message in hex: ");
                    for(int i = 0; i < buffer_pos; i++) {
                        printf("%02X ", buffer[i]);
                    }
                    printf("\n");
                    state = STATE_LOOKING_FOR_PREAMBLE;
                    print_sampling_stats();
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

int main(void) {
    set_process_priority();

    if (mlockall(MCL_CURRENT|MCL_FUTURE) != 0) {
        perror("mlockall failed");
    }

    setup_io();
    INP_GPIO(GPIO_PIN);

    printf("\nOOK Signal Receiver with Enhanced Sampling\n");
    printf("------------------------------------------\n");
    printf("Listening on GPIO %d\n", GPIO_PIN);
    printf("Bit duration: %d ns\n", BIT_DURATION_NS);
    printf("Sampling interval: %d ns\n", SAMPLE_INTERVAL_NS);
    printf("Samples per bit: %d\n", SAMPLES_PER_BIT);
    printf("------------------------------------------\n");
    printf("Press Ctrl+C to stop.\n\n");

    receive_messages();

    return 0;
}