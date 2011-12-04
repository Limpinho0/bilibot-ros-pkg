#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <serial.h>

Serial::Serial()
{
    this->device = "/dev/ttyACM0";
}

Serial::Serial(std::string device)
{
    this->device = device;
}

int Serial::openPort() 
{
    struct termios options;

    this->fd = open(this->device.c_str(), O_RDWR | O_NOCTTY);

    if (this->fd == -1 )
    {
        return -1;
    }
    else
    {
        tcgetattr(this->fd, &options);

        // set baud rate
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);

        // 8N1
        //options.c_cflag &= ~PARENB; // no parity
        //options.c_cflag &= ~CSTOPB; 
        //options.c_cflag &= ~CSIZE; 
        //options.c_cflag |= CS8;
        options.c_cflag = CS8 | CLOCAL | CREAD;
        options.c_cflag &= ~PARENB; // no parity
        options.c_cflag &= ~CSTOPB; 
        options.c_cflag &= ~CSIZE; 
        options.c_iflag = IGNPAR;
        options.c_oflag = 0;
        options.c_lflag = 0;


        tcflush(this->fd, TCIFLUSH);
        tcsetattr(this->fd, TCSANOW, &options);

        return 0;
    }
}

int Serial::closePort()
{
    return close(this->fd);
}

uint8_t Serial::readByte() 
{
    uint8_t c;
    int r = read(this->fd,&c,sizeof(uint8_t));

    if (r != sizeof(uint8_t))
        printf("%d, %s (%d)\n", r, strerror(errno), errno);

    return c;
}

void Serial::writeByte(uint8_t ch) 
{
    int w = write(this->fd, &ch, sizeof(uint8_t));
    if (w != sizeof(uint8_t))
        printf("%d, %s (%d)\n", w, strerror(errno), errno);
}
