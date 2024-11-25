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

#define BCM2711_PERI_BASE 0xFE000000
#define GPIO_BASE (BCM2711_PERI_BASE + 0x200000)
#define PAGE_SIZE 4096
#define BLOCK_SIZE 4096

#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define GPIO_GET(g) (*(gpio+13)&(1<<(g)))

// UART Configuration - must match transmitter
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

// Test Pattern Configuration - must match transmitter
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

// Error Statistics
typedef struct {
    unsigned long total_bytes;
    unsigned long pattern_matches;
    unsigned long pattern_errors;
    unsigned long framing_errors;
    unsigned long bit_errors;
    unsigned long bytes_with_errors;
    uint8_t error_positions[TEST_PATTERN_SIZE];  // Count of errors at each position
    uint8_t bit_error_types[8];  // Count of errors for each bit position
} error_stats_t;

volatile unsigned *gpio;
uint32_t cycles_per_sample;
bool running = true;
error_stats_t stats = {0};

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

// Enhanced error detection
void analyze_received_byte(uint8_t received, uint8_t expected, int position) {
    stats.total_bytes++;
    
    if (received != expected) {
        stats.bytes_with_errors++;
        stats.error_positions[position]++;
        
        // Analyze bit-level errors
        uint8_t xor_result = received ^ expected;
        for (int i = 0; i < 8; i++) {
            if (xor_result & (1 << i)) {
                stats.bit_errors++;
                stats.bit_error_types[i]++;
            }
        }
    }
}

void print_error_statistics(void) {
    printf("\nError Statistics:\n");
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
    
    printf("\nBit Error Distribution:\n");
    for (int i = 0; i < 8; i++) {
        if (stats.bit_error_types[i] > 0) {
            printf("Bit %d: %3u errors (%.2f%%)\n", 
                   i, stats.bit_error_types[i],
                   (stats.bit_error_types[i] * 100.0 / stats.bit_errors));
        }
    }
}

void signal_handler(int signum) {
    running = false;
    print_error_statistics();
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
    uint8_t received_buffer[TEST_PATTERN_SIZE];
    int buffer_position = 0;
    
    while(running) {
        uint8_t received_byte;
        
        if (receive_uart_byte(&received_byte)) {
            analyze_received_byte(received_byte, test_pattern[buffer_position], buffer_position);
            received_buffer[buffer_position] = received_byte;
            
            buffer_position++;
            if (buffer_position >= TEST_PATTERN_SIZE) {
                // Check if we received a complete correct pattern
                if (memcmp(received_buffer, test_pattern, TEST_PATTERN_SIZE) == 0) {
                    stats.pattern_matches++;
                } else {
                    stats.pattern_errors++;
                }
                
                buffer_position = 0;
                
                // Print periodic statistics
                if ((stats.pattern_matches + stats.pattern_errors) % 100 == 0) {
                    // Print periodic statistics
                    if ((stats.pattern_matches + stats.pattern_errors) % 100 == 0) {
                        printf("\nIntermediate Statistics:\n");
                        printf("Patterns Received: %lu\n", stats.pattern_matches + stats.pattern_errors);
                        printf("Error Rate: %.2f%%\n", 
                               (stats.pattern_errors * 100.0) / (stats.pattern_matches + stats.pattern_errors));
                        printf("Bit Error Rate: %.6f%%\n",
                               (stats.bit_errors * 100.0) / (stats.total_bytes * 8));
                    }
                }
            }
        } else {
            stats.framing_errors++;
        }
    }
    
    return 0;
}

// Function to initialize error tracking
void init_error_tracking(void) {
    memset(&stats, 0, sizeof(error_stats_t));
}

// Function to log detailed error information
void log_error_event(uint8_t received, uint8_t expected, int position, uint64_t timestamp) {
    static FILE *error_log = NULL;
    
    if (!error_log) {
        error_log = fopen("uart_errors.log", "w");
        if (!error_log) {
            perror("Failed to open error log");
            return;
        }
        fprintf(error_log, "Timestamp,Position,Expected,Received,BitErrors\n");
    }
    
    fprintf(error_log, "%lu,%d,0x%02X,0x%02X,%d\n",
            timestamp, position, expected, received, __builtin_popcount(received ^ expected));
    fflush(error_log);
}

// Enhanced bit sampling with quality indicator
typedef struct {
    bool bit_value;
    uint8_t confidence;  // 0-100% confidence in the bit value
} bit_sample_t;

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
    // Calculate confidence based on how far from the threshold
    int threshold_distance = abs(high_samples - (SAMPLES_PER_BIT / 2));
    result.confidence = (threshold_distance * 200) / SAMPLES_PER_BIT;  // Scale to 0-100
    
    return result;
}

// Function to print real-time signal quality metrics
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

// Helper function to determine if the signal is reliable
bool is_signal_reliable(void) {
    if (stats.total_bytes < 1000) return true;  // Need minimum sample size
    
    double error_rate = (stats.bytes_with_errors * 100.0) / stats.total_bytes;
    double sync_rate = (stats.pattern_matches * 100.0) / 
                      (stats.pattern_matches + stats.pattern_errors);
    
    return error_rate < 5.0 && sync_rate > 90.0;
}

// Main function with enhanced error reporting
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
        bit_sample_t bit_sample;
        
        if (receive_uart_byte(&received_byte)) {
            // Analyze the received byte against the expected pattern
            analyze_received_byte(received_byte, test_pattern[buffer_position], buffer_position);
            received_buffer[buffer_position] = received_byte;
            
            // Log detailed error information if there's a mismatch
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
                
                // Print periodic quality report
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
    
    print_error_statistics();
    return 0;
}