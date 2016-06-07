package main

/*

// IMU LSM9DS0
// WirePi I2C (Jorge Garza jgarzagu@ucsd.edu)

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
#include <linux/i2c.h>
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


// lsm9d0 Address
#define LSM9DS0_ACC_ADR  (0x3D >> 1) //Adafruit: (0x3B >> 1) Quad: (0x3D >> 1)
#define LSM9DS0_GYRO_ADR (0xD5 >> 1) //Adafruit: (0xD7 >> 1) Quad: (0xD5 >> 1)
#define LSM9DS0_MAG_ADR  (0x3D >> 1) //Adafruit: (0x3B >> 1) Quad  (0x3D >> 1)

// lsm9d0 Registers
#define LSM9DS0_REGISTER_WHO_AM_I_XM        0x0F
#define LSM9DS0_REGISTER_WHO_AM_I_G         0x0F
#define LSM9DS0_XM_ID                       0x49
#define LSM9DS0_G_ID                        0xD4

// lsm9d0 Orientation
#define ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  = X; imu.accADC[PITCH]  = Y; imu.accADC[YAW]  = Z;}
#define GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[ROLL] = X; imu.gyroADC[PITCH] = Y; imu.gyroADC[YAW] = Z;}
#define MAG_ORIENTATION(X, Y, Z)  {imu.magADC[ROLL]  = X; imu.magADC[PITCH]  = Y; imu.magADC[YAW]  = Z;}

// Global Vars
uint8_t rawADC[6];

typedef struct {
  int16_t  magADC[3];
  int16_t  gyroADC[3];
  int16_t  accADC[3];
} imu_t;
imu_t imu;

enum rc {
  ROLL,
  PITCH,
  YAW
};


//
//     I2C
//
//

#define I2C_SLAVE 0x0703
#define BUFFER_LENGTH 32


//// Private vars ///
int fd;
uint8_t rxBuffer[BUFFER_LENGTH];
uint8_t rxBufferIndex = 0;
uint8_t rxBufferLength = 0;

uint8_t txBuffer[BUFFER_LENGTH];
uint8_t txBufferIndex = 0;
uint8_t txBufferLength = 0;

uint8_t transmitting = 0;

int i2c_write_bytes(int file, uint8_t *txBuff, size_t numBytes);
int i2c_read_bytes(int file, uint8_t *rxBuff, size_t numBytes);
void WirePi_begin();
void WirePi_end();
uint8_t WirePi_requestFrom(uint8_t address, uint8_t quantity);
void WirePi_beginTransmission(uint8_t address);
size_t WirePi_write(uint8_t data);
size_t WirePi_write_string(const char *str);
size_t WirePi_write_quantity(uint8_t *data, size_t quantity);
int WirePi_available(void);
int WirePi_read(void);
uint8_t WirePi_endTransmission();


int i2c_write_bytes(int file, uint8_t *txBuff, size_t numBytes)
{
    int bytes_written = 0;

    if (numBytes == 0) {
        return bytes_written;
    } else {
        bytes_written = write(file, txBuff, numBytes);
        if ( bytes_written < 0) {
            // errno == 5 (Input/Output error) means I2C cables may not be connected properly.
            // Make noise about everything else except errno == 5. 
            if (errno != 5 ) {
                fprintf(stderr, "%s(): i2c write error: %s \n",__func__, strerror (errno));
            }
        }
    }

    return bytes_written;
}

int i2c_read_bytes(int file, uint8_t *rxBuff, size_t numBytes)
{
    int bytes_read = 0;

    if (numBytes == 0) {
        return bytes_read;
    } else {
        bytes_read = read(file, rxBuff, numBytes);
        if ( bytes_read < 0) {
            // errno == 5 (Input/Output error) means I2C cables may not be connected properly.
            // Make noise about everything else except errno == 5. 
            if (errno != 5 ) {
                fprintf(stderr, "%s(): i2c read error: %s \n",__func__, strerror (errno));
            }
        }
    }

    return bytes_read;
}

// Initialize the Wire library
void WirePi_begin()
{
    fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "%s(): Error openning I2C channel: %s\n",__func__, strerror (errno));
        exit(1);
    }

    rxBufferIndex = 0;
    rxBufferLength = 0;

    txBufferIndex = 0;
    txBufferLength = 0;

}

void WirePi_end()
{
    close(fd);
    fd = -1;
}


uint8_t WirePi_requestFrom(uint8_t address, uint8_t quantity)
{

    if (fd < 0) {
        fprintf(stderr, "%s(): Initialize I2C first with Wire.begin() \n", __func__);
        exit(1);
    }

    if (ioctl(fd, I2C_SLAVE, address) < 0) {
        fprintf(stderr, "%s(): ioctl error: %s\n",
            __func__, strerror (errno));
        exit(1);
    }

    // clamp to buffer length
    if(quantity > BUFFER_LENGTH){
        quantity = BUFFER_LENGTH;
    }

    // perform blocking read into buffer
    uint8_t read = i2c_read_bytes(fd, rxBuffer, quantity);
    // set rx buffer iterator vars
    rxBufferIndex = 0;
    rxBufferLength = read;

    return read;
}


//Begin a transmission to the I2C slave device with the given address
void WirePi_beginTransmission(uint8_t address)
{

    if (fd < 0) {
        fprintf(stderr, "%s(): Initialize I2C first with Wire.begin() \n", __func__);
        exit(1);
    }

    if (ioctl(fd, I2C_SLAVE, address) < 0) {
        fprintf(stderr, "%s(): ioctl error: %s\n",
            __func__, strerror (errno));
        exit(1);
    }

    // indicate that we are transmitting
    transmitting = 1;
    // reset tx buffer iterator vars
    txBufferIndex = 0;
    txBufferLength = 0;
}

// Writes data to the I2C, returns bytes written.
size_t WirePi_write(uint8_t data)
{

    if (transmitting) {
        // in master transmitter mode
        // don't bother if buffer is full
        if (txBufferLength >= BUFFER_LENGTH) {
          return 0;
        }

        // put byte in tx buffer
        txBuffer[txBufferIndex] = data;
        ++txBufferIndex;
        // update amount in buffer   
        txBufferLength = txBufferIndex;
    } else {
        // in slave send mode
        // reply to master
        i2c_write_bytes(fd, &data, 1);
    }

    return 1;

}

// Writes data to the I2C in form of string, returns bytes written. 
size_t WirePi_write_string(const char *str)
{
    size_t byteswritten = 0;
    size_t i;

    for (i = 0; i < strlen(str) ; i++) {
        // If transmitting data >= BUFFER_LENGTH, then break.
        if (WirePi_write(str[i]) == 0) {
            break;
        }
        byteswritten++;
    }

    return byteswritten;
}


// Writes data to the I2C, returns bytes written. 
size_t WirePi_write_quantity(uint8_t *data, size_t quantity)
{

    size_t byteswritten = 0;
    size_t i;

    if (transmitting) {
        // in master transmitter mode
        for(i = 0; i < quantity; ++i){
            WirePi_write(data[i]);
        }
        byteswritten = quantity;
    } else {
        // in slave send mode
        // reply to master
        byteswritten = i2c_write_bytes(fd, data, quantity);
    }
    
    return byteswritten;
}


int WirePi_available(void)
{
  return rxBufferLength - rxBufferIndex;
}


int WirePi_read(void)
{
  int value = -1;
  
  // get each successive byte on each call
  if (rxBufferIndex < rxBufferLength) {
    value = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;
  }

  return value;
}


uint8_t WirePi_endTransmission()
{
    if (fd < 0) {
        fprintf(stderr, "%s(): Initialize I2C first with Wire.begin() \n", __func__);
        exit(1);
    }

    // Transmit Data 
    uint8_t ret = i2c_write_bytes(fd, txBuffer, txBufferLength);

    // reset tx buffer iterator vars
    txBufferIndex = 0;
    txBufferLength = 0;
    // indicate that we are done transmitting
    transmitting = 0;

    return ret;
}


//
//
//     IMU
//
//

// *** I2C Transmission functions ***
uint8_t i2c_read_reg_to_buf(uint8_t  address, uint8_t  reg, uint8_t *buffer, uint8_t  len)
{
  uint8_t i;

  WirePi_beginTransmission(address);
  WirePi_write(reg);
  WirePi_endTransmission();


  WirePi_requestFrom(address, (uint8_t )len);
  // Wait around until enough data is available
  while (WirePi_available() < len);

  for (i=0; i<len; i++) {
    buffer[i] = WirePi_read();
  }
  WirePi_endTransmission();

  return len;
}

void i2c_writeReg(uint8_t  address, uint8_t  reg, uint8_t  value)
{
    WirePi_beginTransmission(address);
    WirePi_write(reg);
    WirePi_write(value);
    WirePi_endTransmission(); 
}

uint8_t  i2c_readReg(uint8_t  address, uint8_t  reg)
{
  uint8_t value;
  i2c_read_reg_to_buf(address, reg, &value, 1);
  return value;
}

void i2c_getSixRawADC(uint8_t address, uint8_t reg) {
  i2c_read_reg_to_buf(address, reg, rawADC, 6);
}

// *** Acc/Gyro/Mag functions ***
void ACC_init () {
  // Aceleration data rate
  //i2c_writeReg(LSM9DS0_ACC_ADR ,0x20 ,0x47 );  // 25Hz
  i2c_writeReg(LSM9DS0_ACC_ADR ,0x20 ,0x57 ); // 50Hz
  //i2c_writeReg(LSM9DS0_ACC_ADR ,0x20 ,0x67 ); // 100Hz
  //i2c_writeReg(LSM9DS0_ACC_ADR ,0x20 ,0x77 ); // 200Hz
  //i2c_writeReg(LSM9DS0_ACC_ADR ,0x20 ,0x87 ); // 400Hz
  //i2c_writeReg(LSM9DS0_ACC_ADR ,0x20 ,0x97 ); // 800Hz
  //i2c_writeReg(LSM9DS0_ACC_ADR ,0x20 ,0xA7 ); // 1600Hz

  // Acceleration scale **
  i2c_writeReg(LSM9DS0_ACC_ADR ,0x21 ,0x00 ); // 2G
  //i2c_writeReg(LSM9DS0_ACC_ADR ,0x21 ,0x08 ); // 4G
  //i2c_writeReg(LSM9DS0_ACC_ADR ,0x21 ,0x10 ); // 6G
  //i2c_writeReg(LSM9DS0_ACC_ADR ,0x21 ,0x18 ); // 8G
  //i2c_writeReg(LSM9DS0_ACC_ADR ,0x21 ,0x20 ); // 12G
}

#define ACC_DELIMETER 4

void ACC_getADC () {
  i2c_getSixRawADC(LSM9DS0_ACC_ADR, 0x28 | 0x80);

  ACC_ORIENTATION(((rawADC[1]<<8) | rawADC[0])>>ACC_DELIMETER,
                  ((rawADC[3]<<8) | rawADC[2])>>ACC_DELIMETER,
                  ((rawADC[5]<<8) | rawADC[4])>>ACC_DELIMETER);
}

void Gyro_init(){
  // Gyro Enable
  i2c_writeReg(LSM9DS0_GYRO_ADR, 0x20, 0x0F);   //Ctrl reg 1: 100Hz, normal power, XYZ enable

  // Gyro scales
  i2c_writeReg(LSM9DS0_GYRO_ADR, 0x23, 0x00);   //245 dps scale
  //i2c_writeReg(LSM9DS0_GYRO_ADR, 0x23, 0x10);   //500 dps scale
  //i2c_writeReg(LSM9DS0_GYRO_ADR, 0x23, 0x30);   //2000 dps scale
}

#define GYRO_DELIMETER 2

void Gyro_getADC(){
  i2c_getSixRawADC(LSM9DS0_GYRO_ADR, 0x28 | 0x80);
  GYRO_ORIENTATION( ((rawADC[1]<<8) | rawADC[0])>>GYRO_DELIMETER,
                    ((rawADC[3]<<8) | rawADC[2])>>GYRO_DELIMETER,
                    ((rawADC[5]<<8) | rawADC[4])>>GYRO_DELIMETER);
}

void Mag_init() {
  i2c_writeReg(LSM9DS0_MAG_ADR,0x24,0x08);  // [CTRL_REG5_XM] Mag data rate - 12.5 Hz
  i2c_writeReg(LSM9DS0_MAG_ADR,0x25,0x00);  // [CTRL_REG6_XM] Mag scale to +/- 2Ga **
  i2c_writeReg(LSM9DS0_MAG_ADR,0x26,0x00);  // [CTRL_REG7_XM] Continous conversion mode
  i2c_writeReg(LSM9DS0_MAG_ADR,0x23,0x04);  // [CTRL_REG4_XM]
  i2c_writeReg(LSM9DS0_MAG_ADR,0x12,0x09);  // [INT_CTRL_REG_M]
}

#define MAG_DELIMETER 4

void Device_Mag_getADC() {
  i2c_getSixRawADC(LSM9DS0_MAG_ADR,0x08 | 0x80);
      MAG_ORIENTATION( ((rawADC[1]<<8) | rawADC[0]) >> MAG_DELIMETER,
                       ((rawADC[3]<<8) | rawADC[2]) >> MAG_DELIMETER,
                       ((rawADC[5]<<8) | rawADC[4]) >> MAG_DELIMETER);
}



//
//
//     Digital
//
//

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

*/
import "C"
import "fmt"
import "os"

const GPIO4 = 4    // Input
const GPIO17 = 17  // Output just to supply voltage to the pushbutton. 

func main() {
	var timestamp C.struct_timespec

	var id int;
	var chip_detected bool = true;
  
  fmt.Println("LSM9D0 Test");

  //Digital Init
  C.pinMode(GPIO4, C.INPUT);
  C.pinMode(GPIO17, C.OUTPUT);
  C.digitalWrite(GPIO17, C.HIGH);

	//I2C init
	C.WirePi_begin();

	// Detect the IMU chip
	id = int(C.i2c_readReg(C.LSM9DS0_GYRO_ADR, C.LSM9DS0_REGISTER_WHO_AM_I_G));
	if (id != int(C.LSM9DS0_G_ID)) {
		chip_detected = false;
	}
	if (!chip_detected) {
		fmt.Println("Unable to initialize the LSM9DS0. Check your wiring!");
		return;
	}


	// open output file
  f, err := os.Create("output.txt")
  if err != nil {
      panic(err)
  }
  // close fo on exit and check for its returned error
  defer func() {
      if err := f.Close(); err != nil {
          panic(err)
      }
  }()

  // Initialize Acc, Gyro & Mag
	C.ACC_init ();
	C.Gyro_init();
	C.Mag_init();


  fmt.Println("Ready to press button")
  for {
    if (C.digitalRead(GPIO4) == C.LOW) {
      break
    }
  }
  fmt.Println("Button Pressed")



  fmt.Println("Loggin Accelerometer data ...");
	for {
		C.ACC_getADC ();
    C.Gyro_getADC();
		C.clock_gettime(C.CLOCK_REALTIME, &timestamp)
  	buffer := fmt.Sprintf("AccelXYZ, %d, %d, %d, %d, %d\n", C.imu.accADC[C.ROLL],C.imu.accADC[C.PITCH], C.imu.accADC[C.YAW], int(timestamp.tv_sec), int(timestamp.tv_nsec));
  	//fmt.Printf(buffer)
  	// Write to the file
  	f.WriteString(buffer)
  	//C.sleep(1)
	}

}