#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>

#define TX_PIN 17   // GPIO 4
#define RX_PIN 26  // GPIO 26

void send_byte(unsigned char byte) {
    for (int i = 0; i < 8; i++) {
        digitalWrite(TX_PIN, (byte >> i) & 1);
        delayMicroseconds(1);
    }
}

unsigned char receive_byte() {
    unsigned char byte = 0;
    for (int i = 0; i < 8; i++) {
        byte |= (digitalRead(RX_PIN) << i);
        delayMicroseconds(1);
    }
    return byte;
}

int main() {
    // Setup
    if (wiringPiSetupGpio() == -1) {
        printf("WiringPi setup failed\n");
        return 1;
    }

    // Configure pins
    pinMode(TX_PIN, OUTPUT);
    pinMode(RX_PIN, INPUT);
    
    printf("GPIO setup complete\n");

    // Open input and output files
    FILE *infile = fopen("image_data.bin", "rb");
    if (infile == NULL) {
        printf("Could not open input file\n");
        return 1;
    }
    
    FILE *outfile = fopen("received_data.bin", "wb");
    if (outfile == NULL) {
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
    return 0;
}