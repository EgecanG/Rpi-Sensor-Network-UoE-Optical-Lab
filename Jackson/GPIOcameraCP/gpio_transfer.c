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

int setup_gpio() {
    int mem_fd;
    void *gpio_map;

    mem_fd = open("/dev/gpiomem", O_RDWR|O_SYNC);
    if (mem_fd < 0) {
        printf("Error opening /dev/gpiomem\n");
        return -1;
    }

    gpio_map = mmap(NULL, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, GPIO_BASE);
    close(mem_fd);

    if (gpio_map == MAP_FAILED) {
        printf("mmap error\n");
        return -1;
    }

    gpio = (volatile unsigned *)gpio_map;

    // Configure pins
    *(gpio + (TX_PIN/10)) &= ~(7<<((TX_PIN%10)*3));
    *(gpio + (TX_PIN/10)) |=  (1<<((TX_PIN%10)*3));  // Set TX as output
    *(gpio + (RX_PIN/10)) &= ~(7<<((RX_PIN%10)*3));  // Set RX as input

    return 0;
}

void transfer_bit(unsigned char bit, unsigned char *received) {
    // Send bit
    if (bit) {
        *(gpio + 7) = 1 << TX_PIN;   // Set high
    } else {
        *(gpio + 10) = 1 << TX_PIN;  // Set low
    }
    
    usleep(100);  // Wait for signal to stabilize
    
    // Read received bit
    *received = (*(gpio + 13) >> RX_PIN) & 1;
    
    usleep(100);  // Wait before next bit
}

int transfer_byte(unsigned char byte, unsigned char *received_byte) {
    unsigned char received_bit;
    *received_byte = 0;
    
    // Transfer each bit
    for (int i = 0; i < 8; i++) {
        transfer_bit((byte >> i) & 1, &received_bit);
        *received_byte |= (received_bit << i);
    }
    
    return 0;
}

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

    // Transfer data
    unsigned char byte, received_byte;
    long bytes_transferred = 0;
    clock_t start = clock();

    while (fread(&byte, 1, 1, infile) == 1) {
        transfer_byte(byte, &received_byte);
        fwrite(&received_byte, 1, 1, outfile);
        
        bytes_transferred++;
        if (bytes_transferred % 1024 == 0) {
            printf("\rProgress: %ld%%", (bytes_transferred * 100) / size);
            fflush(stdout);
        }
    }

    clock_t end = clock();
    double time_taken = ((double)(end - start)) / CLOCKS_PER_SEC;
    double speed = (bytes_transferred / (1024.0 * 1024.0)) / time_taken;

    printf("\nTransfer complete!\n");
    printf("Bytes transferred: %ld\n", bytes_transferred);
    printf("Time taken: %.2f seconds\n", time_taken);
    printf("Speed: %.2f MB/s\n", speed);

    fclose(infile);
    fclose(outfile);
    munmap((void*)gpio, BLOCK_SIZE);
    return 0;
}