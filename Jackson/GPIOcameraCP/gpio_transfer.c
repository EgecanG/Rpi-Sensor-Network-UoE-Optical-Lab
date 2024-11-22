#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>

// GPIO Memory addresses
#define BCM2708_PERI_BASE   0xFE000000  // For RPi 4
#define GPIO_BASE           (BCM2708_PERI_BASE + 0x200000)
#define PAGE_SIZE           (4*1024)
#define BLOCK_SIZE          (4*1024)

// GPIO pins
#define TX_PIN 17
#define RX_PIN 26

// GPIO setup registers
#define GPIO_INP_GPIO(g)    *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define GPIO_OUT_GPIO(g)    *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define GPIO_SET_GPIO(g)    *(gpio+7) = 1<<(g)
#define GPIO_CLR_GPIO(g)    *(gpio+10) = 1<<(g)
#define GPIO_GET_GPIO(g)    (*(gpio+13)&(1<<g))

// Global pointer to GPIO memory
volatile unsigned *gpio;

// Function to set up memory mapping
int setup_gpio() {
    int mem_fd;
    void *gpio_map;

    if ((mem_fd = open("/dev/gpiomem", O_RDWR|O_SYNC)) < 0) {
        printf("Error: Unable to open /dev/gpiomem\n");
        return -1;
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
        printf("Error: mmap failed\n");
        return -1;
    }

    gpio = (volatile unsigned *)gpio_map;
    return 0;
}

void send_byte(unsigned char byte) {
    for (int i = 0; i < 8; i++) {
        if (byte & (1 << i)) {
            GPIO_SET_GPIO(TX_PIN);
        } else {
            GPIO_CLR_GPIO(TX_PIN);
        }
        // Small delay (you can adjust this)
        for(volatile int j = 0; j < 150; j++) { }
    }
}

unsigned char receive_byte() {
    unsigned char byte = 0;
    for (int i = 0; i < 8; i++) {
        if (GPIO_GET_GPIO(RX_PIN)) {
            byte |= (1 << i);
        }
        // Small delay (you can adjust this)
        for(volatile int j = 0; j < 150; j++) { }
    }
    return byte;
}

int main() {
    // Setup GPIO
    if (setup_gpio() < 0) {
        return 1;
    }

    // Setup GPIO pins
    GPIO_INP_GPIO(TX_PIN);  // Must set as input first
    GPIO_OUT_GPIO(TX_PIN);  // Then set as output
    GPIO_INP_GPIO(RX_PIN);  // Set RX as input
    
    printf("GPIO setup complete\n");

    // Open files
    FILE *infile = fopen("image_data.bin", "rb");
    if (!infile) {
        printf("Could not open input file\n");
        return 1;
    }

    FILE *outfile = fopen("received_data.bin", "wb");
    if (!outfile) {
        printf("Could not open output file\n");
        fclose(infile);
        return 1;
    }

    // Get file size
    fseek(infile, 0, SEEK_END);
    long size = ftell(infile);
    fseek(infile, 0, SEEK_SET);
    printf("File size: %ld bytes\n", size);

    // Transfer data
    clock_t start = clock();
    unsigned char byte;
    long bytes_transferred = 0;

    while (fread(&byte, 1, 1, infile) == 1) {
        send_byte(byte);
        unsigned char received = receive_byte();
        fwrite(&received, 1, 1, outfile);
        
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

    // Cleanup
    fclose(infile);
    fclose(outfile);
    munmap((void*)gpio, BLOCK_SIZE);
    return 0;
}