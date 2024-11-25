#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
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
#include <stdbool.h>

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
#define STOP_BITS 1

// Receiver Enhancement Parameters
#define SAMPLES_PER_BIT 16
#define NOISE_FILTER_SAMPLES 3
#define START_BIT_THRESHOLD (SAMPLES_PER_BIT * 3 / 4)
#define DATA_BIT_THRESHOLD (SAMPLES_PER_BIT / 2)
#define GPIO_PIN 4

// Test Pattern Configuration
#define TEST_PATTERN_SIZE 64
const uint8_t test_pattern[TEST_PATTERN_SIZE] = {
    0x55, 0xAA, 0x00, 0xFF, 0x0F, 0xF0, 0x33, 0xCC,
    0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0,
    'H', 'e', 'l', 'l', 'o', ' ', 'W', 'o',
    'r', 'l', 'd', '!', 0x0D, 0x0A, 0x00, 0xFF,
    0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,
    0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01,
    0x1F, 0x2F, 0x3F, 0x4F, 0x5F, 0x6F, 0x7F, 0x8F,
    0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8
};

// Error Statistics Structure
typedef struct {
    unsigned long total_bytes;
    unsigned long pattern_matches;
    unsigned long pattern_errors;
    unsigned long framing_errors;
    unsigned long bit_errors;
    unsigned long bytes_with_errors;
    uint8_t error_positions[TEST_PATTERN_SIZE];
    uint8_t bit_error_types[8];
} error_stats_t;

// Bit sampling structure
typedef struct {
    bool bit_value;
    uint8_t confidence;
} bit_sample_t;

// Global variables
volatile unsigned *gpio;
uint32_t cycles_per_sample;
bool running = true;
error_stats_t stats = {0};
FILE *error_log = NULL;

// Function prototypes
static inline uint64_t get_cycles(void);
uint64_t get_cpu_freq(void);
void setup_io(void);
bool filter_noise(void);
bool detect_start_bit(void);
bit_sample_t enhanced_sample_bit(void);
bool receive_uart_byte(uint8_t *byte);
void analyze_received_byte(uint8_t received, uint8_t expected, int position);
void print_error_statistics(void);
void init_error_tracking(void);
void log_error_event(uint8_t received, uint8_t expected, int position, uint64_t timestamp);
void print_signal_quality(void);
bool is_signal_reliable(void);
void signal_handler(int signum);

// Inline assembly for cycle counting
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
    cycles_per_sample = (cpu_freq * BIT_DURATION_NS) / (1000000000ULL * SAMPLES_PER_BIT);
    
    printf("CPU Frequency: %lu Hz\n", cpu_freq);
    printf("Cycles per sample: %u\n", cycles_per_sample);
}

bool filter_noise(void) {
    int high_count = 0;
    for (int i = 0; i < NOISE_FILTER_SAMPLES; i++) {
        if (GPIO_GET(GPIO_PIN)) {
            high_count++;
        }
        for (volatile int j = 0; j < 10; j++) {
            asm volatile("nop");
        }
    }
    return high_count > (NOISE_FILTER_SAMPLES / 2);
}

bool detect_start_bit(void) {
    int low_samples = 0;
    uint64_t start_cycle = get_cycles();
    
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

bit_sample_t enhanced_sample_bit(void) {
    int high_samples = 0;
    uint64_t start_cycle = get_cycles();
    bit_sample_t result;
    
    for (int i = 0; i < SAMPLES_PER_BIT; i++) {
        while ((get_cycles() - start_cycle) < (cycles_per_sample * i)) {
            asm volatile("nop");
        }
        if (filter_noise()) {
            high_samples++;
        }
    }
    
    result.bit_value = high_samples >= DATA_BIT_THRESHOLD;
    int threshold_distance = abs(high_samples - (SAMPLES_PER_BIT / 2));
    result.confidence = (threshold_distance * 200) / SAMPLES_PER_BIT;
    
    return result;
}

bool receive_uart_byte(uint8_t *byte) {
    if (!detect_start_bit()) {
        return false;
    }
    
    uint8_t data = 0;
    bit_sample_t bit_sample;
    
    for (int i = 0; i < DATA_BITS; i++) {
        bit_sample = enhanced_sample_bit();
        if (bit_sample.bit_value) {
            data |= (1 << i);
        }
    }
    
    // Verify stop bit(s)
    for (int i = 0; i < STOP_BITS; i++) {
        bit_sample = enhanced_sample_bit();
        if (!bit_sample.bit_value) {
            return false;
        }
    }
    
    *byte = data;
    return true;
}

void analyze_received_byte(uint8_t received, uint8_t expected, int position) {
    stats.total_bytes++;
    
    if (received != expected) {
        stats.bytes_with_errors++;
        stats.error_positions[position]++;
        
        uint8_t xor_result = received ^ expected;
        for (int i = 0; i < 8; i++) {
            if (xor_result & (1 << i)) {
                stats.bit_errors++;
                stats.bit_error_types[i]++;
            }
        }
    }
}

void init_error_tracking(void) {
    memset(&stats, 0, sizeof(error_stats_t));
    error_log = fopen("uart_errors.log", "w");
    if (error_log) {
        fprintf(error_log, "Timestamp,Position,Expected,Received,BitErrors\n");
    }
}

void log_error_event(uint8_t received, uint8_t expected, int position, uint64_t timestamp) {
    if (error_log) {
        fprintf(error_log, "%lu,%d,0x%02X,0x%02X,%d\n",
                timestamp, position, expected, received,
                __builtin_popcount(received ^ expected));
        fflush(error_log);
    }
}

void print_signal_quality(void) {
    printf("\nSignal Quality Metrics:\n");
    printf("Short-term Error Rate: %.2f%%\n", 
           (stats.bytes_with_errors * 100.0) / (stats.total_bytes ? stats.total_bytes : 1));
    
    printf("Bit Error Distribution:\n");
    printf("LSB [");
    for (int i = 0; i < 8; i++) {
        int error_rate = (stats.bit_error_types[i] * 100) / 
                        (stats.total_bytes ? stats.total_bytes : 1);
        printf("%3d%%|", error_rate);
    }
    printf("] MSB\n");
    
    printf("Pattern Sync Rate: %.1f%%\n",
           (stats.pattern_matches * 100.0) / 
           ((stats.pattern_matches + stats.pattern_errors) ? 
            (stats.pattern_matches + stats.pattern_errors) : 1));
}

void print_error_statistics(void) {
    printf("\nFinal Error Statistics:\n");
    printf("Total Bytes Received: %lu\n", stats.total_bytes);
    printf("Complete Pattern Matches: %lu\n", stats.pattern_matches);
    printf("Pattern Errors: %lu\n", stats.pattern_errors);
    printf("Framing Errors: %lu\n", stats.framing_errors);
    printf("Total Bit Errors: %lu\n", stats.bit_errors);
    printf("Bytes With Errors: %lu (%.2f%%)\n", 
           stats.bytes_with_errors,
           (stats.total_bytes > 0) ? 
           (stats.bytes_with_errors * 100.0 / stats.total_bytes) : 0);
    
    printf("\nError Distribution by Position:\n");
    for (int i = 0; i < TEST_PATTERN_SIZE; i++) {
        if (stats.error_positions[i] > 0) {
            printf("Position %2d: %3u errors (%.2f%%) [Byte: 0x%02X]\n", 
                   i, stats.error_positions[i],
                   (stats.error_positions[i] * 100.0 / (stats.total_bytes / TEST_PATTERN_SIZE)),
                   test_pattern[i]);
        }
    }
}

bool is_signal_reliable(void) {
    if (stats.total_bytes < 1000) return true;
    
    double error_rate = (stats.bytes_with_errors * 100.0) / stats.total_bytes;
    double sync_rate = (stats.pattern_matches * 100.0) / 
                      (stats.pattern_matches + stats.pattern_errors);
    
    return error_rate < 5.0 && sync_rate > 90.0;
}

void signal_handler(int signum) {
    running = false;
    if (error_log) {
        fclose(error_log);
    }
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
    INP_GPIO(GPIO_PIN);
    init_error_tracking();

    printf("Enhanced UART Test Pattern Receiver\n");
    printf("Configuration:\n");
    printf("- Baud Rate: %d\n", BAUD_RATE);
    printf("- Samples per Bit: %d\n", SAMPLES_PER_BIT);
    printf("- Pattern Size: %d bytes\n", TEST_PATTERN_SIZE);
    printf("Monitoring started...\n\n");

    uint8_t received_buffer[TEST_PATTERN_SIZE];
    int buffer_position = 0;
    time_t last_quality_report = time(NULL);
    
    while(running) {
        uint8_t received_byte;
        
        if (receive_uart_byte(&received_byte)) {
            analyze_received_byte(received_byte, test_pattern[buffer_position], buffer_position);
            received_buffer[buffer_position] = received_byte;
            
            if (received_byte != test_pattern[buffer_position]) {
                log_error_event(received_byte, test_pattern[buffer_position], 
                              buffer_position, get_cycles());
            }
            
            buffer_position++;
            if (buffer_position >= TEST_PATTERN_SIZE) {
                if (memcmp(received_buffer, test_pattern, TEST_PATTERN_SIZE) == 0) {
                    stats.pattern_matches++;
                } else {
                    stats.pattern_errors++;
                }
                
                buffer_position = 0;
                
                time_t current_time = time(NULL);
                if (current_time - last_quality_report >= 5) {
                    print_signal_quality();
                    last_quality_report = current_time;
                    
                    if (!is_signal_reliable()) {
                        printf("\nWARNING: Signal quality has degraded!\n");
                    }
                }
            }
        } else {
            stats.framing_errors++;
        }
    }
    
    
    // Print final statistics
    printf("\nReceiver stopped. Generating final report...\n");
    print_error_statistics();
    print_signal_quality();
    
    // Clean up
    if (error_log) {
        fclose(error_log);
    }
    
    printf("\nError log saved to uart_errors.log\n");
    return 0;
}