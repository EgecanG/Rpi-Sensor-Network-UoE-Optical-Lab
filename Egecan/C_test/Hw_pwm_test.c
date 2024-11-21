#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#define BCM2711_PERI_BASE 0xFE000000
#define GPIO_BASE (BCM2711_PERI_BASE + 0x200000)
#define BLOCK_SIZE 4096

// GPIO setup macros
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))

volatile unsigned *gpio;

// Nanosecond delay function
static inline void delay_ns(unsigned int ns) {
    struct timespec ts = {
        .tv_sec = 0,
        .tv_nsec = ns
    };
    nanosleep(&ts, NULL);
}

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
}

void send_bit(int bit)
{
    if (bit) {
        *(gpio + 7) = 1 << 17;  // Set GPIO 17 high (GPSET0)
    } else {
        *(gpio + 10) = 1 << 17; // Set GPIO 17 low (GPCLR0)
    }
    delay_ns(1000);  // 1µs delay for 1Mbps
}

void send_byte(unsigned char byte)
{
    // Send start bit (1)
    send_bit(1);
    
    // Send 8 data bits, MSB first
    for (int i = 7; i >= 0; i--) {
        send_bit((byte >> i) & 0x01);
    }
    
    // Send stop bit (1)
    send_bit(1);
    
    // Small gap between bytes
    send_bit(0);
}

void send_test_pattern()
{
    // 1. Alternating 1s and 0s
    printf("Sending alternating pattern (0x55)...\n");
    for(int i = 0; i < 100; i++) {
        send_byte(0x55);  // 01010101
    }
    
    // 2. All ones
    printf("Sending all ones (0xFF)...\n");
    for(int i = 0; i < 100; i++) {
        send_byte(0xFF);  // 11111111
    }
    
    // 3. All zeros
    printf("Sending all zeros (0x00)...\n");
    for(int i = 0; i < 100; i++) {
        send_byte(0x00);  // 00000000
    }

    // 4. Counter pattern
    printf("Sending counter pattern...\n");
    for(int i = 0; i < 256; i++) {
        send_byte((unsigned char)i);
    }
}

int main(int argc, char **argv)
{
    // Set up GPIO
    setup_io();

    // Configure GPIO 17 as output
    INP_GPIO(17);
    OUT_GPIO(17);

    printf("1Mbps Baseband OOK Transmitter\n");
    printf("Using GPIO 17 (Pin 11)\n");
    printf("Data Rate: 1 Mbps\n");
    printf("Format: Start(1) + 8 Data + Stop(1) + Gap(0)\n");

    // Main loop
    while(1) {
        send_test_pattern();
        usleep(1000);  // 1ms gap between patterns
    }

    return 0;
}