#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>

#define BCM2708_PERI_BASE   0xFE000000
#define GPIO_BASE           (BCM2708_PERI_BASE + 0x200000)
#define BLOCK_SIZE          (4*1024)

#define TX_PIN 17
#define RX_PIN 26

volatile unsigned *gpio;

// Inline assembly for precise delay
static inline void delay_cycles(int cycles) {
    for(int i = 0; i < cycles; i++) {
        asm volatile("nop");
    }
}

static inline void gpio_set(void) {
    *(gpio + 7) = 1 << TX_PIN;
}

static inline void gpio_clear(void) {
    *(gpio + 10) = 1 << TX_PIN;
}

static inline int gpio_read(void) {
    return (*(gpio + 13) >> RX_PIN) & 1;
}

int setup_gpio() {
    int mem_fd;
    void *gpio_map;

    mem_fd = open("/dev/gpiomem", O_RDWR|O_SYNC);
    if (mem_fd < 0) return -1;

    gpio_map = mmap(NULL, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, GPIO_BASE);
    close(mem_fd);
    if (gpio_map == MAP_FAILED) return -1;

    gpio = (volatile unsigned *)gpio_map;
    
    // Configure pins - optimized setup
    *(gpio + (TX_PIN/10)) &= ~(7<<((TX_PIN%10)*3));
    *(gpio + (TX_PIN/10)) |=  (1<<((TX_PIN%10)*3));
    *(gpio + (RX_PIN/10)) &= ~(7<<((RX_PIN%10)*3));
    
    return 0;
}

void transfer_byte(unsigned char byte, unsigned char *received_byte) {
    *received_byte = 0;
    
    // Unrolled loop for maximum speed
    gpio_clear();
    delay_cycles(1);
    if(byte & 0x01) gpio_set(); else gpio_clear();
    delay_cycles(1);
    *received_byte |= (gpio_read() << 0);
    
    if(byte & 0x02) gpio_set(); else gpio_clear();
    delay_cycles(1);
    *received_byte |= (gpio_read() << 1);
    
    if(byte & 0x04) gpio_set(); else gpio_clear();
    delay_cycles(1);
    *received_byte |= (gpio_read() << 2);
    
    if(byte & 0x08) gpio_set(); else gpio_clear();
    delay_cycles(1);
    *received_byte |= (gpio_read() << 3);
    
    if(byte & 0x10) gpio_set(); else gpio_clear();
    delay_cycles(1);
    *received_byte |= (gpio_read() << 4);
    
    if(byte & 0x20) gpio_set(); else gpio_clear();
    delay_cycles(1);
    *received_byte |= (gpio_read() << 5);
    
    if(byte & 0x40) gpio_set(); else gpio_clear();
    delay_cycles(1);
    *received_byte |= (gpio_read() << 6);
    
    if(byte & 0x80) gpio_set(); else gpio_clear();
    delay_cycles(1);
    *received_byte |= (gpio_read() << 7);
    
    gpio_clear();
}

#define BUFFER_SIZE 4096

int main() {
    if (setup_gpio() < 0) return 1;

    FILE *infile = fopen("image_data.bin", "rb");
    FILE *outfile = fopen("received_data.bin", "wb");
    
    if (!infile || !outfile) {
        printf("File error\n");
        return 1;
    }

    // Get file size
    fseek(infile, 0, SEEK_END);
    long size = ftell(infile);
    fseek(infile, 0, SEEK_SET);
    printf("File size: %ld bytes\n", size);

    // Allocate buffers for block transfer
    unsigned char *in_buffer = malloc(BUFFER_SIZE);
    unsigned char *out_buffer = malloc(BUFFER_SIZE);
    
    if (!in_buffer || !out_buffer) {
        printf("Memory allocation error\n");
        return 1;
    }

    // Transfer data with timing measurement
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC_RAW, &start);
    
    long bytes_transferred = 0;
    size_t bytes_read;
    
    while ((bytes_read = fread(in_buffer, 1, BUFFER_SIZE, infile)) > 0) {
        for (size_t i = 0; i < bytes_read; i++) {
            transfer_byte(in_buffer[i], &out_buffer[i]);
        }
        fwrite(out_buffer, 1, bytes_read, outfile);
        
        bytes_transferred += bytes_read;
        printf("\rProgress: %ld%%", (bytes_transferred * 100) / size);
        fflush(stdout);
    }

    clock_gettime(CLOCK_MONOTONIC_RAW, &end);
    
    double time_taken = ((end.tv_sec - start.tv_sec) * 1000000000.0 + 
                        (end.tv_nsec - start.tv_nsec)) / 1000000000.0;
    double speed = (bytes_transferred / (1024.0 * 1024.0)) / time_taken;

    printf("\nTransfer complete!\n");
    printf("Bytes transferred: %ld\n", bytes_transferred);
    printf("Time taken: %.3f seconds\n", time_taken);
    printf("Speed: %.2f MB/s\n", speed);

    free(in_buffer);
    free(out_buffer);
    fclose(infile);
    fclose(outfile);
    munmap((void*)gpio, BLOCK_SIZE);
    return 0;
}