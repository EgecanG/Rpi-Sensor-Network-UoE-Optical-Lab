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
#include <signal.h>
#include <stdbool.h>

#define BCM2711_PERI_BASE 0xFE000000
#define GPIO_BASE (BCM2711_PERI_BASE + 0x200000)
#define PAGE_SIZE 4096
#define BLOCK_SIZE 4096

#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define GPIO_GET(g) (*(gpio+13)&(1<<(g)))

#define BAUD_RATE 9600
#define BIT_DURATION_NS (1000000000 / BAUD_RATE)
#define SAMPLE_DELAY (BIT_DURATION_NS / 2)  // Sample in middle of bit

#define GPIO_PIN 4
#define MAX_MESSAGE 256

volatile unsigned *gpio;
uint32_t cycles_per_bit;
bool running = true;

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
    
    printf("CPU Frequency: %lu Hz\n", cpu_freq);
    printf("Cycles per bit: %u\n", cycles_per_bit);
}

bool wait_for_start_bit(void) {
    // Wait for line to be idle (high)
    while (!GPIO_GET(GPIO_PIN)) {
        asm volatile("nop");
    }
    
    // Wait for falling edge (start bit)
    while (GPIO_GET(GPIO_PIN)) {
        asm volatile("nop");
    }
    
    // Wait for half a bit time to reach middle of start bit
    uint64_t start = get_cycles();
    while ((get_cycles() - start) < (cycles_per_bit / 2)) {
        asm volatile("nop");
    }
    
    return true;
}

bool receive_byte(uint8_t *byte) {
    uint8_t data = 0;
    
    if (!wait_for_start_bit()) {
        return false;
    }
    
    uint64_t bit_start = get_cycles();
    
    // Read 8 data bits
    for (int i = 0; i < 8; i++) {
        // Wait until middle of bit
        while ((get_cycles() - bit_start) < cycles_per_bit) {
            asm volatile("nop");
        }
        
        // Sample the bit
        if (GPIO_GET(GPIO_PIN)) {
            data |= (1 << i);
        }
        
        bit_start += cycles_per_bit;
    }
    
    // Wait for and verify stop bit
    while ((get_cycles() - bit_start) < cycles_per_bit) {
        asm volatile("nop");
    }
    
    if (!GPIO_GET(GPIO_PIN)) {
        return false;  // Invalid stop bit
    }
    
    *byte = data;
    return true;
}

void signal_handler(int signum) {
    running = false;
}

int main(void) {
    signal(SIGINT, signal_handler);
    
    struct sched_param sp = { .sched_priority = sched_get_priority_max(SCHED_FIFO) };
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        perror("Failed to set real-time priority");
    }

    setup_io();
    INP_GPIO(GPIO_PIN);

    printf("UART Receiver Started\n");
    printf("Baud Rate: %d\n", BAUD_RATE);
    printf("Waiting for data...\n\n");

    char message[MAX_MESSAGE];
    int pos = 0;
    
    while(running) {
        uint8_t received_byte;
        
        if (receive_byte(&received_byte)) {
            if (received_byte == '\r' || received_byte == '\n') {
                if (pos > 0) {
                    message[pos] = '\0';
                    printf("Received: %s\n", message);
                    pos = 0;
                }
            } else {
                if (pos < MAX_MESSAGE - 1) {
                    message[pos++] = received_byte;
                }
            }
        }
    }
    
    printf("\nReceiver stopped.\n");
    return 0;
}