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

// Timing constants
#define BIT_DURATION_NS 1000    // 1000ns = 1μs = 1Mbps
#define MESSAGE_INTERVAL_MS 10   // 10ms between messages

volatile unsigned *gpio;
volatile unsigned *gpio_set;
volatile unsigned *gpio_clr;
unsigned int gpio_bit;

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

void delay_ms(unsigned int ms) {
    struct timespec ts = {
        .tv_sec = ms / 1000,
        .tv_nsec = (ms % 1000) * 1000000
    };
    nanosleep(&ts, NULL);
}

void transmit_bit(int bit) {
    if (bit) {
        *gpio_set = gpio_bit;  // Set high for 1
    } else {
        *gpio_clr = gpio_bit;  // Set low for 0
    }
    delay_ns(BIT_DURATION_NS);
}

void transmit_byte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        transmit_bit((byte >> i) & 0x01);
    }
}

void transmit_message(const uint8_t *data, size_t length) {
    // Transmit preamble (8 ones)
    for (int i = 0; i < 8; i++) {
        transmit_bit(1);
    }
    
    // Transmit data
    for (size_t i = 0; i < length; i++) {
        transmit_byte(data[i]);
    }
    
    // Transmit postamble (8 zeros)
    for (int i = 0; i < 8; i++) {
        transmit_bit(0);
    }
    
    // Ensure GPIO is low after transmission
    *gpio_clr = gpio_bit;
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

    const char *message = argv[1];
    size_t message_length = strlen(message);
    
    printf("Transmitting message: \"%s\"\n", message);
    printf("Message length: %zu bytes\n", message_length);
    printf("Data rate: 1 Mbps (1μs per bit)\n");
    printf("Message interval: %d ms\n", MESSAGE_INTERVAL_MS);
    printf("Press Ctrl+C to stop.\n\n");
    
    unsigned long message_count = 0;
    
    while(1) {
        message_count++;
        printf("\rMessages sent: %lu", message_count);
        fflush(stdout);
        
        // Transmit the message
        transmit_message((const uint8_t *)message, message_length);
        
        // Wait for the specified interval
        delay_ms(MESSAGE_INTERVAL_MS);
    }

    return 0;
}