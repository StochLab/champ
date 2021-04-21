#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <termios.h>
#include <signal.h>
#include <pthread.h>
#include <sys/poll.h>

#define BAUD B1500000 


/**
* \brief finding file descriptor
*/
int scan_devices(int j);

/**
* \brief Initializing uart 
*/
int UARTInit(int fd);


/**
* \brief Writting to serial port
*/
int UARTWrite(int serial_port, uint8_t msg[], int size);

/**
* \brief Reading from serial port
*/
int UARTRead(int serial_port, uint8_t* read_msg, int size);

/**
* \brief Flushing the port
*/
void flush(int fd);

/**
* \brief Flushing the port
*/
void GPIOInit(int pin);

