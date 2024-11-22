#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>

#define BCM2708_PERI_BASE   0xFE000000
#define GPIO_BASE           (BCM2708_PERI_BASE + 0x200000)
#define BLOCK_SIZE          (4*1024)
#define BUFFER_SIZE         (4*1024)

#define TX_PIN 17
#define RX_PIN 26
#define TARGET_SPEED 1048576  // 1 MB/s

// Direct GPIO access for speed
volatile unsigned *gpio;

static inline void gpio_set(void) {
    *(gpio + 7) = 1 << TX_PIN;
}

static inline void gpio_clear(void) {
    *(gpio + 10) = 1 << TX_PIN;
}

static inline int gpio_read(void) {
    return (*(gpio + 13) >> RX_PIN) & 1;
}

static inline void delay_cycles(int cycles) {
    for(int i = 0; i < cycles; i++) {
        asm volatile("nop");
    }
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
    
    // Configure pins
    *(gpio + (TX_PIN/10)) &= ~(7<<((TX_PIN%10)*3));
    *(gpio + (TX_PIN/10)) |=  (1<<((TX_PIN%10)*3));
    *(gpio + (RX_PIN/10)) &= ~(7<<((RX_PIN%10)*3));
    
    return 0;
}

void send_byte_fast(unsigned char byte) {
    // Unrolled loop for maximum speed
    (byte & 0x01) ? gpio_set() : gpio_clear();
    delay_cycles(2);
    (byte & 0x02) ? gpio_set() : gpio_clear();
    delay_cycles(2);
    (byte & 0x04) ? gpio_set() : gpio_clear();
    delay_cycles(2);
    (byte & 0x08) ? gpio_set() : gpio_clear();
    delay_cycles(2);
    (byte & 0x10) ? gpio_set() : gpio_clear();
    delay_cycles(2);
    (byte & 0x20) ? gpio_set() : gpio_clear();
    delay_cycles(2);
    (byte & 0x40) ? gpio_set() : gpio_clear();
    delay_cycles(2);
    (byte & 0x80) ? gpio_set() : gpio_clear();
    delay_cycles(2);
}

unsigned char receive_byte_fast() {
    unsigned char byte = 0;
    byte |= gpio_read(); delay_cycles(2);
    byte |= gpio_read() << 1; delay_cycles(2);
    byte |= gpio_read() << 2; delay_cycles(2);
    byte |= gpio_read() << 3; delay_cycles(2);
    byte |= gpio_read() << 4; delay_cycles(2);
    byte |= gpio_read() << 5; delay_cycles(2);
    byte |= gpio_read() << 6; delay_cycles(2);
    byte |= gpio_read() << 7; delay_cycles(2);
    return byte;
}

int main() {
    if (setup_gpio() < 0) return 1;
    printf("GPIO setup complete\n");

    unsigned char *buffer = malloc(BUFFER_SIZE);
    unsigned char *recv_buffer = malloc(BUFFER_SIZE);
    if (!buffer || !recv_buffer) return 1;

    struct timespec start_time, current_time;
    clock_gettime(CLOCK_MONOTONIC, &start_time);
    long total_bytes = 0;
    int iterations = 0;

    while (1) {
        // Open new files for each iteration
        FILE *infile = fopen("image_data.bin", "rb");
        FILE *outfile = fopen("received_data.bin", "wb");
        if (!infile || !outfile) break;

        size_t bytes_read;
        while ((bytes_read = fread(buffer, 1, BUFFER_SIZE, infile)) > 0) {
            for (size_t i = 0; i < bytes_read; i++) {
                send_byte_fast(buffer[i]);
                recv_buffer[i] = receive_byte_fast();
            }
            fwrite(recv_buffer, 1, bytes_read, outfile);
            total_bytes += bytes_read;

            // Check if 10 seconds have elapsed
            clock_gettime(CLOCK_MONOTONIC, &current_time);
            double elapsed = (current_time.tv_sec - start_time.tv_sec) + 
                           (current_time.tv_nsec - start_time.tv_nsec) / 1e9;
            
            double current_rate = total_bytes / elapsed / 1048576.0;
            printf("\rTime: %.2fs, Rate: %.2f MB/s", elapsed, current_rate);
            fflush(stdout);

            if (elapsed >= 10.0) {
                printf("\n\nFinal Results:\n");
                printf("Total bytes: %ld\n", total_bytes);
                printf("Time: %.2f seconds\n", elapsed);
                printf("Average rate: %.2f MB/s\n", current_rate);
                printf("Iterations completed: %d\n", iterations);
                
                free(buffer);
                free(recv_buffer);
                fclose(infile);
                fclose(outfile);
                munmap((void*)gpio, BLOCK_SIZE);
                return 0;
            }
        }
        
        fclose(infile);
        fclose(outfile);
        iterations++;
    }

    free(buffer);
    free(recv_buffer);
    munmap((void*)gpio, BLOCK_SIZE);
    return 0;
}