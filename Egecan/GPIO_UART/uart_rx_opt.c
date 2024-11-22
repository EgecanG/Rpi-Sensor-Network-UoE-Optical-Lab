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
#define GPIO_GET(g) (*(gpio+13)&(1<<(g)))

// UART Configuration
#define BAUD_RATE 100000
#define BIT_DURATION_NS (1000000000 / BAUD_RATE)
#define DATA_BITS 8
#define USE_PARITY false
#define PARITY_EVEN true
#define STOP_BITS 1

// Receiver Enhancement Parameters
#define SAMPLES_PER_BIT 16  // Oversampling rate
#define NOISE_FILTER_SAMPLES 3  // Number of samples for noise filtering
#define START_BIT_THRESHOLD (SAMPLES_PER_BIT * 3 / 4)  // 75% of samples must be low for start bit
#define DATA_BIT_THRESHOLD (SAMPLES_PER_BIT / 2)  // 50% of samples for data bit decision
#define MAX_MESSAGE_SIZE 1024
#define GPIO_PIN 4

// Statistics tracking
typedef struct {
    unsigned long total_bytes;
    unsigned long framing_errors;
    unsigned long noise_errors;
    unsigned long parity_errors;
    unsigned long successful_reads;
    uint64_t last_signal_quality;
} uart_stats_t;

volatile unsigned *gpio;
uint32_t cycles_per_sample;
bool running = true;
uart_stats_t stats = {0};

// Signal quality tracking
#define SIGNAL_QUALITY_WINDOW 100
uint8_t signal_quality_samples[SIGNAL_QUALITY_WINDOW];
int signal_quality_index = 0;

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

// Enhanced noise filtering with multiple samples
bool filter_noise(void) {
    int high_count = 0;
    for (int i = 0; i < NOISE_FILTER_SAMPLES; i++) {
        if (GPIO_GET(GPIO_PIN)) {
            high_count++;
        }
        // Small delay between samples
        for (volatile int j = 0; j < 10; j++) {
            asm volatile("nop");
        }
    }
    return high_count > (NOISE_FILTER_SAMPLES / 2);
}

// Update signal quality metrics
void update_signal_quality(bool expected, bool received) {
    signal_quality_samples[signal_quality_index] = (expected == received) ? 1 : 0;
    signal_quality_index = (signal_quality_index + 1) % SIGNAL_QUALITY_WINDOW;
    
    // Calculate current signal quality percentage
    int quality_sum = 0;
    for (int i = 0; i < SIGNAL_QUALITY_WINDOW; i++) {
        quality_sum += signal_quality_samples[i];
    }
    stats.last_signal_quality = (quality_sum * 100) / SIGNAL_QUALITY_WINDOW;
}

// Enhanced start bit detection with multiple samples
bool detect_start_bit(void) {
    int low_samples = 0;
    uint64_t start_cycle = get_cycles();
    
    // Wait for stable low period
    for (int i = 0; i < SAMPLES_PER_BIT; i++) {
        while ((get_cycles() - start_cycle) < (cycles_per_sample * i)) {
            asm volatile("nop");
        }
        if (!filter_noise()) {
            low_samples++;
        }
    }
    
    return low_samples >= START_BIT_THRESHOLD;
}

// Enhanced bit sampling with majority voting
bool sample_bit(void) {
    int high_samples = 0;
    uint64_t start_cycle = get_cycles();
    
    for (int i = 0; i < SAMPLES_PER_BIT; i++) {
        while ((get_cycles() - start_cycle) < (cycles_per_sample * i)) {
            asm volatile("nop");
        }
        if (filter_noise()) {
            high_samples++;
        }
    }
    
    return high_samples >= DATA_BIT_THRESHOLD;
}

bool receive_uart_byte(uint8_t *byte) {
    uint8_t data = 0;
    bool success = true;
    
    // Wait for and verify start bit
    if (!detect_start_bit()) {
        stats.framing_errors++;
        return false;
    }
    
    // Sample each data bit
    for (int i = 0; i < DATA_BITS; i++) {
        bool bit = sample_bit();
        if (bit) {
            data |= (1 << i);
        }
        update_signal_quality(bit, sample_bit()); // Check bit stability
    }
    
    // Handle parity if enabled
    if (USE_PARITY) {
        bool parity_bit = sample_bit();
        bool calculated_parity = __builtin_parity(data) ^ !PARITY_EVEN;
        if (parity_bit != calculated_parity) {
            stats.parity_errors++;
            success = false;
        }
    }
    
    // Verify stop bit(s)
    for (int i = 0; i < STOP_BITS; i++) {
        if (!sample_bit()) {
            stats.framing_errors++;
            success = false;
            break;
        }
    }
    
    if (success) {
        *byte = data;
        stats.successful_reads++;
    }
    stats.total_bytes++;
    
    return success;
}

void print_statistics(void) {
    printf("\nReceiver Statistics:\n");
    printf("Total Bytes: %lu\n", stats.total_bytes);
    printf("Successful Reads: %lu (%.2f%%)\n", 
           stats.successful_reads,
           (stats.total_bytes > 0) ? (stats.successful_reads * 100.0 / stats.total_bytes) : 0);
    printf("Framing Errors: %lu\n", stats.framing_errors);
    printf("Parity Errors: %lu\n", stats.parity_errors);
    printf("Current Signal Quality: %lu%%\n", stats.last_signal_quality);
}

void signal_handler(int signum) {
    running = false;
    print_statistics();
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
    cycles_per_sample = (cpu_freq * BIT_DURATION_NS) / (1000000000ULL * SAMPLES_PER_BIT);
    
    printf("CPU Frequency: %lu Hz\n", cpu_freq);
    printf("Cycles per sample: %u\n", cycles_per_sample);
}

int main(void) {
    char received_message[MAX_MESSAGE_SIZE];
    size_t message_length = 0;
    
    signal(SIGINT, signal_handler);
    
    // Set CPU affinity to core 3 (assuming 4 cores)
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(3, &set);
    if (sched_setaffinity(0, sizeof(set), &set) == -1) {
        perror("Failed to set CPU affinity");
    }
    
    // Set real-time priority
    struct sched_param sp = { .sched_priority = sched_get_priority_max(SCHED_FIFO) };
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        perror("Failed to set real-time priority");
    }

    if (mlockall(MCL_CURRENT|MCL_FUTURE) != 0) {
        perror("mlockall failed");
    }

    setup_io();
    INP_GPIO(GPIO_PIN);

    printf("Enhanced UART receiver starting:\n");
    printf("Configuration:\n");
    printf("- Baud Rate: %d\n", BAUD_RATE);
    printf("- Samples per Bit: %d\n", SAMPLES_PER_BIT);
    printf("- Noise Filter Samples: %d\n", NOISE_FILTER_SAMPLES);
    printf("Waiting for data...\n\n");
    
    while (running) {
        uint8_t received_byte;
        
        if (receive_uart_byte(&received_byte)) {
            received_message[message_length++] = received_byte;
            
            if (received_byte == '\0' || received_byte == '\n' || 
                message_length >= MAX_MESSAGE_SIZE - 1) {
                received_message[message_length] = '\0';
                printf("Received (%lu bytes): %s\n", message_length, received_message);
                message_length = 0;
                
                // Print periodic statistics
                if (stats.total_bytes % 100 == 0) {
                    print_statistics();
                }
            }
        }
    }
    
    return 0;
}