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
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))

#define BIT_DURATION_NS 250     // 250ns per bit

volatile unsigned *gpio;
volatile unsigned *gpio_set;
volatile unsigned *gpio_clr;
unsigned int gpio_bit;
uint32_t cycles_per_bit;

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

void set_process_priority(void) {
    if (setpriority(PRIO_PROCESS, 0, -20) != 0) {
        perror("Failed to set process priority");
    }

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
    gpio_set = gpio + 7;
    gpio_clr = gpio + 10;
    gpio_bit = 1 << 4;

    uint64_t cpu_freq = get_cpu_freq();
    cycles_per_bit = (cpu_freq * BIT_DURATION_NS) / 1000000000ULL;
    
    printf("CPU Frequency: %lu Hz\n", cpu_freq);
    printf("Cycles per bit: %u\n", cycles_per_bit);
}

inline void transmit_bit(int bit) {
    uint64_t start_cycle = get_cycles();
    
    *(gpio_set + (bit ^ 1) * 3) = gpio_bit;
    
    while ((get_cycles() - start_cycle) < cycles_per_bit) {
        asm volatile("nop");
    }
}

inline void transmit_byte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        transmit_bit((byte >> i) & 0x01);
    }
}

void transmit_message(const uint8_t *data, size_t length) {
    // Transmit alternating 1-0 preamble (8 bits total)
    for (int i = 0; i < 8; i++) {
        transmit_bit(i & 1);  // Will transmit 1,0,1,0,1,0,1,0
    }
    
    // Transmit a special start sequence (11110000)
    transmit_byte(0x0F);
    
    // Transmit data
    for (size_t i = 0; i < length; i++) {
        transmit_byte(data[i]);
    }
    
    // Transmit end sequence (00001111)
    transmit_byte(0xF0);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        printf("Usage: %s <message>\n", argv[0]);
        return 1;
    }

    set_process_priority();

    if (mlockall(MCL_CURRENT|MCL_FUTURE) != 0) {
        perror("mlockall failed");
    }

    setup_io();

    INP_GPIO(4);
    OUT_GPIO(4);

    const char *message = argv[1];
    size_t message_length = strlen(message);
    
    printf("Starting continuous transmission:\n");
    printf("Message: \"%s\"\n", message);
    printf("Message length: %zu bytes\n", message_length);
    printf("Target bit duration: %d ns\n", BIT_DURATION_NS);
    printf("Using alternating 1-0 preamble pattern\n");
    printf("Press Ctrl+C to stop.\n\n");

    while(1) {
        transmit_message((const uint8_t *)message, message_length);
    }

    return 0;
}