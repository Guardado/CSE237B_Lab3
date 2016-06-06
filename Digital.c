#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/mman.h>
#include <string.h>
#include <time.h>
#include <ctype.h>
#include <sys/ioctl.h>
#include <limits.h>
#include <limits.h>
#include <stdarg.h>
#include <linux/types.h>
#include <unistd.h>

// Pin logic states
#define HIGH 0x1
#define LOW  0x0

// Pin modes
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
#define INPUT_PULLDOWN 0x3


// BCM2708 Registers for GPIO (Do not put them in .h)
#define BCM2708_PERI_BASE   0x20000000
#define GPIO_BASE           (BCM2708_PERI_BASE + 0x200000)
#define OFFSET_FSEL         0   // 0x0000
#define OFFSET_SET          7   // 0x001c / 4
#define OFFSET_CLR          10  // 0x0028 / 4
#define OFFSET_PINLEVEL     13  // 0x0034 / 4
#define OFFSET_PULLUPDN     37  // 0x0094 / 4
#define OFFSET_PULLUPDNCLK  38  // 0x0098 / 4
#define GPIO_FSEL_INPUT     0   // Pin Input mode
#define GPIO_FSEL_OUTPUT    1   // Pin Output mode
#define PAGE_SIZE  (4*1024)
#define BLOCK_SIZE (4*1024)
static volatile uint32_t *gpio_map = NULL;
static uint8_t open_gpiomem_flag = 0;
char GPIO_DRIVER_NAME[] = "/dev/gpiomem";


void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
int digitalRead(uint8_t pin);

// Sets pin (gpio) mode as INPUT/INTPUT_PULLUP/INTPUT_PULLDOWN/OUTPUT
void pinMode(uint8_t pin, uint8_t mode)
{
    int mem_fd;
    uint8_t *gpio_mem;
    int clk_offset = OFFSET_PULLUPDNCLK + (pin/32);
    int shift_offset = (pin%32);
    int offset = OFFSET_FSEL + (pin/10);
    int shift = (pin%10)*3;

    // Initialize gpiomem only once
    if (open_gpiomem_flag == 0) {
        if ((mem_fd = open(GPIO_DRIVER_NAME, O_RDWR|O_SYNC) ) < 0) {
            fprintf(stderr, "%s(): gpio driver %s: %s\n",__func__, 
                GPIO_DRIVER_NAME, strerror (errno));
            exit(1);
        }

        if ((gpio_mem = (uint8_t *) malloc(BLOCK_SIZE + (PAGE_SIZE-1))) == NULL) {
            fprintf(stderr, "%s(): gpio error: %s\n",__func__, strerror (errno));
            exit(1);
        }

        if ((uint32_t)gpio_mem % PAGE_SIZE) {
            gpio_mem += PAGE_SIZE - ((uint32_t)gpio_mem % PAGE_SIZE);
        }

        gpio_map = (uint32_t *)mmap( (void *)gpio_mem, BLOCK_SIZE, 
            PROT_READ|PROT_WRITE, MAP_SHARED|MAP_FIXED, mem_fd, GPIO_BASE);

        if ((uint32_t)gpio_map < 0) {
            fprintf(stderr, "%s(): gpio error: %s\n",__func__, strerror (errno));
            exit(1);
        }

        // gpiomem initialized correctly
        open_gpiomem_flag = 1;
    }


    // Set resistor mode PULLUP, PULLDOWN or PULLOFF resitor (OUTPUT always PULLOFF)
    if (mode == INPUT_PULLDOWN) {
       *(gpio_map+OFFSET_PULLUPDN) = (*(gpio_map+OFFSET_PULLUPDN) & ~3) | 0x01;
    } else if (mode == INPUT_PULLUP) {
       *(gpio_map+OFFSET_PULLUPDN) = (*(gpio_map+OFFSET_PULLUPDN) & ~3) | 0x02;
    } else { // mode == PULLOFF
       *(gpio_map+OFFSET_PULLUPDN) &= ~3;
    }
    usleep(1);
    *(gpio_map+clk_offset) = 1 << shift_offset;
    usleep(1);
    *(gpio_map+OFFSET_PULLUPDN) &= ~3;
    *(gpio_map+clk_offset) = 0;

    // Set pin mode INPUT/OUTPUT
    if (mode == OUTPUT) {
        *(gpio_map+offset) = (*(gpio_map+offset) & ~(7<<shift)) | (GPIO_FSEL_OUTPUT<<shift);
    } else { // mode == INPUT or INPUT_PULLUP or INPUT_PULLDOWN
        *(gpio_map+offset) = (*(gpio_map+offset) & ~(7<<shift)) | (GPIO_FSEL_INPUT<<shift);
    }

}

// Sets a pin (gpio) output to 1 or 0
void digitalWrite(uint8_t pin, uint8_t val)
{
    int offset;
    if (val) { // value == HIGH
        offset = OFFSET_SET + (pin / 32);
    } else {    // value == LOW
        offset = OFFSET_CLR + (pin / 32);
    }
    *(gpio_map+offset) = 1 << pin % 32;
}

// Returns the value of a pin (gpio) input (1 or 0)
int digitalRead(uint8_t pin)
{
   int offset, value, mask;
   offset = OFFSET_PINLEVEL + (pin/32);
   mask = (1 << pin%32);
   value = *(gpio_map+offset) & mask;
   return (value) ? HIGH : LOW;
}



#define GPIO4 4    // Input
#define GPIO17 17  // Output just to supply voltage to the pushbutton. 

int main () {
    // Time vars
	time_t t = time(NULL);
	struct tm *tm;
	struct timespec tr;
    char sendBuff[1025];

	pinMode(GPIO4, INPUT);
  	pinMode(GPIO17, OUTPUT);
  	digitalWrite(GPIO17, HIGH);

	while(1) {
		printf("Ready to press button\n");
		while (digitalRead(GPIO4) == HIGH);

		// Get current time
		clock_gettime(CLOCK_REALTIME, &tr);
		tm = localtime(&t);
		strftime(sendBuff,80,"%d:%b:%Y:%H:%M", tm);
		sprintf(sendBuff, "%s:%llu:%llu\n", sendBuff, (long long unsigned int) tr.tv_sec, (long long unsigned int) tr.tv_nsec);
		printf("Button Pressed Time = %s \n", sendBuff);

		// To set current time in nanoseconds use clock_settime()

		sleep(1); // Added this line
	}
	return (0);
}