/*
 * polulu_servo.h
 *
 *  Created on: Oct 19, 2012
 *      Author: arouthu
 */

#ifndef POLOLU_SERVO_H_
#define POLOLU_SERVO_H_
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#ifdef _WIN32
#define O_NOCTTY 0
#else
#include <termios.h>
#endif

int openPort(const char * portName, unsigned int baudRate)
{
    // Open the Maestro's virtual COM port.
    //const char * device = "\\\\.\\USBSER000";  // Windows, "\\\\.\\COM6" also works
    const char * device = portName; // Linux
    //const char * device = "/dev/cu.usbmodem00034567"; // Mac OS X
    int fd = open(device, O_RDWR | O_NOCTTY);
    if (fd == -1)
    {
        perror(device);
        return 1;
    }

#ifndef _WIN32
    struct termios options;
    tcgetattr(fd, &options);
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    options.c_oflag &= ~(ONLCR | OCRNL);
    tcsetattr(fd, TCSANOW, &options);
#endif

    // close(fd);
    return fd;
}
void closePort(int fd)
{
    close(fd);
}

int maestroGetPosition(int fd, unsigned char channel, unsigned short * position)
{
    unsigned char command[2];
    unsigned char response[2];

    // Compose the command.
    command[0] = 0x90;
    command[1] = channel;

    // Send the command to the device.
    if (write(fd, command, sizeof(command)) == -1)
    {
        perror("error writing");
        return -1;
    }

    // Read the response from the device.
    if (read(fd, response, 2) != 2)
    {
        perror("error reading");
        return -1;
    }

    // Convert the bytes received in to a position.
    *position = response[0] + 256 * response[1];

    return 1;
}

int maestroSetTarget(int fd, unsigned char channel, unsigned short target)
{
    //unsigned char command[6];
    // Compose the command.
    unsigned char command[] = { 0x84, channel, target & 0x7F, target >> 7 & 0x7F }; //maestro
    //unsigned char command[] ={ 0xAA, 0x0C, 0x04, channel, target & 0x7F, target >> 7 & 0x7F };//ttl

    // Send the command to the device.
    if (write(fd, command, sizeof(command)) == -1)
    {
        perror("error writing");
        return -1;
    }
    return 1;
}

#endif /* POLULU_SERVO_H_ */
