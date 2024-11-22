// gpio_transfer.c
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>

// GPIO setup
#define BCM2708_PERI_BASE 0x3F000000  // Raspberry Pi 3/4
#define GPIO_BASE (BCM2708_PERI_BASE + 0x200000)
#define BLOCK_SIZE (4*1024)

// GPIO pins
#define TX_PIN 17
#define RX_PIN 26

// GPIO setup registers
#define GPIO_INPUT  0
#define GPIO_OUTPUT 1

// Volatile pointer for GPIO memory mapping
volatile uint32_t *gpio;

// GPIO access functions
void setup_gpio_memory() {
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
        printf("mmap failed\n");
        exit(-1);
    }

    gpio = (volatile uint32_t *)gpio_map;
}

void setup_gpio() {
    // Set TX as output
    int reg = TX_PIN / 10;
    int shift = (TX_PIN % 10) * 3;
    gpio[reg] = (gpio[reg] & ~(7 << shift)) | (1 << shift);

    // Set RX as input
    reg = RX_PIN / 10;
    shift = (RX_PIN % 10) * 3;
    gpio[reg] = (gpio[reg] & ~(7 << shift));
}

// Fast GPIO operations
inline void gpio_write(int value) {
    if (value)
        gpio[7] = 1 << TX_PIN;  // Set bit (output high)
    else
        gpio[10] = 1 << TX_PIN; // Clear bit (output low)
}

inline int gpio_read() {
    return (gpio[13] >> RX_PIN) & 1;
}

// Fast byte transfer functions
void send_byte(unsigned char byte) {
    for (int i = 0; i < 8; i++) {
        gpio_write((byte >> i) & 1);
        // Minimal delay using NOP
        asm volatile("nop");
    }
}

unsigned char receive_byte() {
    unsigned char byte = 0;
    for (int i = 0; i < 8; i++) {
        byte |= (gpio_read() << i);
        // Minimal delay using NOP
        asm volatile("nop");
    }
    return byte;
}

// Main transfer function
void transfer_data(unsigned char *data, size_t size) {
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    for (size_t i = 0; i < size; i++) {
        send_byte(data[i]);
        
        if (i % 1024 == 0) {
            clock_gettime(CLOCK_MONOTONIC, &end);
            double elapsed = (end.tv_sec - start.tv_sec) + 
                           (end.tv_nsec - start.tv_nsec) / 1e9;
            double rate = (i / elapsed) / (1024 * 1024);
            printf("\rTransfer rate: %.2f MB/s", rate);
            fflush(stdout);
        }
    }

    clock_gettime(CLOCK_MONOTONIC, &end);
    double total_time = (end.tv_sec - start.tv_sec) + 
                       (end.tv_nsec - start.tv_nsec) / 1e9;
    double final_rate = (size / total_time) / (1024 * 1024);
    printf("\nTransfer complete. Average rate: %.2f MB/s\n", final_rate);
}

int main() {
    // Setup
    setup_gpio_memory();
    setup_gpio();

    // Test speed
    printf("Testing maximum transfer speed...\n");
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);
    
    // Send 1MB of test data
    size_t test_size = 1024 * 1024;
    unsigned char *test_data = malloc(test_size);
    for (size_t i = 0; i < test_size; i++) {
        test_data[i] = i & 0xFF;
    }

    // Transfer test data
    transfer_data(test_data, test_size);

    // Cleanup
    free(test_data);
    return 0;
}
