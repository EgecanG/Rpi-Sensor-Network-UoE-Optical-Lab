#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <time.h>
#include <string.h>

#define BCM2708_PERI_BASE   0xFE000000
#define GPIO_BASE           (BCM2708_PERI_BASE + 0x200000)
#define BLOCK_SIZE          (4*1024)

#define TX_PIN 17
#define RX_PIN 26

volatile unsigned *gpio;

// Setup GPIO
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
    *(gpio + (TX_PIN/10)) &= ~(7<<((TX_PIN%10)*3));  // Set to input first
    *(gpio + (TX_PIN/10)) |=  (1<<((TX_PIN%10)*3));  // Set to output
    *(gpio + (RX_PIN/10)) &= ~(7<<((RX_PIN%10)*3));  // Set to input

    return 0;
}

// Send one bit
void send_bit(int bit) {
    if (bit) {
        *(gpio + 7) = 1 << TX_PIN;   // Set high
    } else {
        *(gpio + 10) = 1 << TX_PIN;  // Set low
    }
}

// Read one bit
int read_bit() {
    return (*(gpio + 13) >> RX_PIN) & 1;
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        printf("Usage: %s [send|receive]\n", argv[0]);
        return 1;
    }

    if (setup_gpio() < 0) {
        return 1;
    }

    if (strcmp(argv[1], "send") == 0) {
        // Read data from stdin (binary)
        unsigned char byte;
        while (fread(&byte, 1, 1, stdin) == 1) {
            // Send each bit
            for (int i = 0; i < 8; i++) {
                int bit = (byte >> i) & 1;
                send_bit(bit);
                usleep(1000);  // 1ms delay
            }
        }
    }
    else if (strcmp(argv[1], "receive") == 0) {
        // Receive bits and write to stdout
        unsigned char byte = 0;
        int bit_count = 0;
        
        while (1) {
            int bit = read_bit();
            byte |= (bit << bit_count);
            bit_count++;
            
            if (bit_count == 8) {
                fwrite(&byte, 1, 1, stdout);
                byte = 0;
                bit_count = 0;
            }
            usleep(1000);  // 1ms delay
        }
    }

    return 0;
}