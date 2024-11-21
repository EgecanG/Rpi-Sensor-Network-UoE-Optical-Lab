#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#define BCM2711_PERI_BASE 0xFE000000
#define PWM_BASE (BCM2711_PERI_BASE + 0x20C000)
#define CLOCK_BASE (BCM2711_PERI_BASE + 0x101000)
#define GPIO_BASE (BCM2711_PERI_BASE + 0x200000)

#define PWM_CTL  0
#define PWM_RNG1 4
#define PWM_DAT1 5

#define PWMCLK_CNTL 40
#define PWMCLK_DIV  41

#define BLOCK_SIZE 4096

#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

volatile unsigned *pwm;
volatile unsigned *clk;
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
    void *gpio_map, *pwm_map, *clk_map;

    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC)) < 0) {
        printf("Failed to open /dev/mem\n");
        exit(-1);
    }

    gpio_map = mmap(NULL, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, GPIO_BASE);
    pwm_map = mmap(NULL, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, PWM_BASE);
    clk_map = mmap(NULL, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, CLOCK_BASE);

    close(mem_fd);

    if (gpio_map == MAP_FAILED || pwm_map == MAP_FAILED || clk_map == MAP_FAILED) {
        printf("mmap error\n");
        exit(-1);
    }

    gpio = (volatile unsigned *)gpio_map;
    pwm = (volatile unsigned *)pwm_map;
    clk = (volatile unsigned *)clk_map;
}

void setup_pwm(int carrier_freq)
{
    // Stop PWM
    *(pwm + PWM_CTL) = 0;

    // Stop PWM clock
    *(clk + PWMCLK_CNTL) = 0x5A000000 | (1 << 5);
    usleep(10);

    // Calculate parameters for carrier frequency
    int range = 8;  // Smaller range for higher frequencies
    int divisor = 500000000 / (carrier_freq * range);
    
    printf("PWM Carrier Configuration:\n");
    printf("Carrier Frequency: %d Hz\n", carrier_freq);
    printf("Range: %d\n", range);
    printf("Divisor: %d\n", divisor);
    printf("Actual Frequency: %d Hz\n", 500000000 / (range * divisor));
    
    *(clk + PWMCLK_DIV) = 0x5A000000 | (divisor << 12);
    *(clk + PWMCLK_CNTL) = 0x5A000011;
    usleep(10);

    *(pwm + PWM_RNG1) = range;
    *(pwm + PWM_DAT1) = range/2;  // 50% duty cycle
}

void pwm_output(int on)
{
    if (on) {
        *(pwm + PWM_CTL) = 0x81;  // Enable PWM
    } else {
        *(pwm + PWM_CTL) = 0;     // Disable PWM
    }
}

void send_byte_ook(unsigned char byte)
{
    // Send start bit (1)
    pwm_output(1);
    delay_ns(1000);  // 1µs per bit for 1Mbps
    
    // Send 8 data bits
    for (int i = 0; i < 8; i++) {
        int bit = (byte >> (7-i)) & 0x01;
        pwm_output(bit);
        delay_ns(1000);  // 1µs delay
    }
    
    // Send stop bit (1)
    pwm_output(1);
    delay_ns(1000);
    
    // Turn off carrier and add small gap
    pwm_output(0);
    delay_ns(1000);
}

// Function to send a test pattern
void send_test_pattern(void)
{
    // Alternating 1s and 0s
    printf("Sending alternating pattern...\n");
    unsigned char alt_pattern = 0x55;  // 01010101
    for(int i = 0; i < 100; i++) {
        send_byte_ook(alt_pattern);
    }
    
    // All ones
    printf("Sending all ones...\n");
    for(int i = 0; i < 100; i++) {
        send_byte_ook(0xFF);
    }
    
    // All zeros
    printf("Sending all zeros...\n");
    for(int i = 0; i < 100; i++) {
        send_byte_ook(0x00);
    }
}

int main(int argc, char **argv)
{
    // Set up memory regions
    setup_io();

    // Configure GPIO 18 for PWM
    INP_GPIO(18);
    SET_GPIO_ALT(18, 5);

    // Set up carrier frequency (10MHz for good sampling of 1Mbps data)
    int carrier_freq = 10000000;  // 10MHz carrier
    setup_pwm(carrier_freq);

    printf("1Mbps OOK Transmitter\n");
    printf("Carrier Frequency: %d Hz\n", carrier_freq);
    printf("Data Rate: 1 Mbps\n");

    while(1) {
        // Send test pattern continuously
        send_test_pattern();
        
        // Small gap between patterns
        usleep(1000);
    }

    return 0;
}