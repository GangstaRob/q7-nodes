#include <ros/ros.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
using namespace std;

#define Numel(x) sizeof(x)/sizeof(x[0])
struct spi_ioc_transfer tr[256];

string convert_int(int n) {
    stringstream ss;
    ss << n;
    return ss.str();
}

void SpiWriteRead(int length, const char *device, uint8_t *msg)
{
// Initialize parameters
    uint8_t mode = 0;
    uint8_t bits = 8;
    uint32_t speed = 32000;

// Open device
    int fd = open(device, O_RDWR);
    if (fd < 0) {
        ROS_ERROR("Can't open device");
        exit(1);
    }
// SPI mode
    int ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
    if (ret < 0) {
        ROS_ERROR("Can't set spi mode");
        exit(1);
    }
// Bits per word
    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret < 0) {
        ROS_ERROR("Can't set bits per word");
        exit(1);
    }
// Max speed, 1ms per 32bit command --> 32kHz
    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret < 0) {
        ROS_ERROR("Can't set max speed");
        exit(1);
    }

    unsigned char tx[256];
    unsigned char rx[256];
    for (int i = 0; i < length; i++) {
        tx[i] = 0x00;
        rx[i] = 0x00;
    }

    printf("tx: ");
    for(int i;i<Numel(tx);i++){
        cout << convert_int(tx[i]);
    }
    printf("\n");

    for (int i = 0; i < length; i++) {
        tr[i].tx_buf = (unsigned long)&tx[i];
        tr[i].rx_buf = (unsigned long)&rx[i];
        tr[i].len = 1;
        tr[i].delay_usecs = 1000;
        tr[i].speed_hz = speed;
        tr[i].bits_per_word = 8;
    }
    ret = ioctl(fd, SPI_IOC_MESSAGE(length), tr);
    if (ret < 1) {
        ROS_ERROR("Error read or write - ioctl");
        exit(1);
    }
    int i=0;
    printf("rx: ");
    for(i;i<Numel(rx);i++){
        cout << convert_int(rx[i]);
    }
    printf("\n");
// Close device
    close(fd);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "spiInterface");
    ros::NodeHandle nh;
    ros::Duration duration(1. / 24.);
// Length of byte stream
    int length = 2;
    int count = 10;
    sleep(2);
    while (ros::ok() && count > 0) {
        unsigned char rightMsg[] = { 0x00, 0x77, 0x00, 0x77, 0x00, 0x77, 0x00 };
        const char *dev0 = "/dev/spidev1.0";
        SpiWriteRead(length, dev0, rightMsg);
        duration.sleep();
        count--;
    }
    return 0;
}