#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>

// PWM registers addresses for RPi 4
#define BCM2711_PERI_BASE 0xFE000000
#define PWM_BASE (BCM2711_PERI_BASE + 0x20C000)  // PWM controller base
#define CLOCK_BASE (BCM2711_PERI_BASE + 0x101000) // Clock controller base
#define GPIO_BASE (BCM2711_PERI_BASE + 0x200000)  // GPIO base

#define PWM_CTL  0  // Control register
#define PWM_RNG1 4  // Range register for channel 1
#define PWM_DAT1 5  // Data register for channel 1

#define PWMCLK_CNTL 40  // Clock control
#define PWMCLK_DIV  41  // Clock divisor

#define BLOCK_SIZE 4096

// GPIO setup macros
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

volatile unsigned *pwm;
volatile unsigned *clk;
volatile unsigned *gpio;

void setup_io()
{
    int mem_fd;
    void *gpio_map, *pwm_map, *clk_map;

    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC)) < 0) {
        printf("Failed to open /dev/mem\n");
        exit(-1);
    }

    // Map GPIO
    gpio_map = mmap(NULL, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, GPIO_BASE);
    
    // Map PWM
    pwm_map = mmap(NULL, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, mem_fd, PWM_BASE);
    
    // Map CLK
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

void setup_pwm(int frequency)
{
    // Stop PWM
    *(pwm + PWM_CTL) = 0;

    // Stop PWM clock
    *(clk + PWMCLK_CNTL) = 0x5A000000 | (1 << 5);
    usleep(10);

    // Calculate parameters
    int range = 32;  // Adjust this for frequency control
    int divisor = 500000000 / (frequency * range);
    
    // Print debug info
    printf("PWM Configuration:\n");
    printf("Requested Frequency: %d Hz\n", frequency);
    printf("Range: %d\n", range);
    printf("Calculated Divisor: %d\n", divisor);
    printf("Actual Frequency: %d Hz\n", 500000000 / (range * divisor));
    
    // Set PWM clock divider
    *(clk + PWMCLK_DIV) = 0x5A000000 | (divisor << 12);
    
    // Start PWM clock
    *(clk + PWMCLK_CNTL) = 0x5A000011;
    usleep(10);

    // Setup PWM
    *(pwm + PWM_RNG1) = range;
    *(pwm + PWM_DAT1) = range/2;  // 50% duty cycle
    
    // Start PWM
    *(pwm + PWM_CTL) = 0x81;
}

int main(int argc, char **argv)
{
    // Set up memory regions to access GPIO and PWM
    setup_io();

    // GPIO 18 (Pin 12) in PWM mode Alt5
    INP_GPIO(18);
    SET_GPIO_ALT(18, 5);

    // Set PWM frequency (e.g., 10MHz)
    int frequency = 10000000;  // 10MHz
    setup_pwm(frequency);

    printf("Generating %dHz PWM signal on GPIO18 (Pin 12)\n", frequency);
    printf("Press Ctrl+C to stop\n");

    while(1) {
        sleep(1);
    }

    return 0;
}