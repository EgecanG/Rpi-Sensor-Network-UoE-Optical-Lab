#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#include <string.h>

#define BCM2711_PERI_BASE 0xFE000000
#define GPIO_BASE (BCM2711_PERI_BASE + 0x200000)
#define PAGE_SIZE 4096
#define BLOCK_SIZE 4096

#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))

volatile unsigned *gpio;
volatile unsigned *gpio_set;
volatile unsigned *gpio_clr;
unsigned int gpio_bit;

// Function prototypes
void setup_io(void);
void delay_ns(unsigned int ns);
void transmit_bit(int bit, unsigned int bit_duration_ns);
void transmit_byte(uint8_t byte, unsigned int bit_duration_ns);
void transmit_data(const uint8_t *data, size_t length, unsigned int bit_duration_ns);

void setup_io()
{
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
    
    // Initialize global pointers to SET and CLEAR registers
    gpio_set = gpio + 7;    // GPSET0
    gpio_clr = gpio + 10;   // GPCLR0
    gpio_bit = 1 << 17;     // GPIO 17
}

void delay_ns(unsigned int ns) {
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC_RAW, &start);
    
    do {
        clock_gettime(CLOCK_MONOTONIC_RAW, &end);
        uint64_t delta_ns = (end.tv_sec - start.tv_sec) * 1000000000ULL + 
                           (end.tv_nsec - start.tv_nsec);
        if (delta_ns >= ns)
            break;
    } while (1);
}

// Transmit a single bit
void transmit_bit(int bit, unsigned int bit_duration_ns) {
    if (bit) {
        *gpio_set = gpio_bit;  // Set high for 1
    } else {
        *gpio_clr = gpio_bit;  // Set low for 0
    }
    delay_ns(bit_duration_ns);
}

// Transmit a byte, LSB first
void transmit_byte(uint8_t byte, unsigned int bit_duration_ns) {
    for (int i = 0; i < 8; i++) {
        transmit_bit((byte >> i) & 0x01, bit_duration_ns);
    }
}

// Transmit arbitrary data
void transmit_data(const uint8_t *data, size_t length, unsigned int bit_duration_ns) {
    printf("Transmitting %zu bytes at %u ns per bit\n", length, bit_duration_ns);
    printf("Data rate: %.2f kbps\n", 1000000.0f / bit_duration_ns);
    
    for (size_t i = 0; i < length; i++) {
        transmit_byte(data[i], bit_duration_ns);
    }
}

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: %s <message>\n", argv[0]);
        return 1;
    }

    // Set up gpio pointer for direct register access
    setup_io();

    // Configure GPIO 17 as output
    INP_GPIO(17);
    OUT_GPIO(17);

    // Convert the message to bytes
    const char *message = argv[1];
    size_t message_length = strlen(message);
    
    // Example: 100 microseconds per bit (10 kbps)
    unsigned int bit_duration_ns = 100000;  
    
    // Transmit start sequence (optional)
    for (int i = 0; i < 8; i++) {
        transmit_bit(1, bit_duration_ns);  // Preamble of 8 ones
    }
    
    // Transmit the actual message
    transmit_data((const uint8_t *)message, message_length, bit_duration_ns);
    
    // Transmit stop sequence (optional)
    for (int i = 0; i < 8; i++) {
        transmit_bit(0, bit_duration_ns);  // Postamble of 8 zeros
    }

    return 0;
}