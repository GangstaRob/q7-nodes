#include <ros/ros.h>
#include "Command.h"
#include <iomanip>
#include <sstream>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
using namespace std;

#define Numel(x) sizeof(x)/sizeof(x[0])
my_pkg::Command myfunction(my_pkg::Command);
struct spi_ioc_transfer tr[256];

string convert_int(int n) {
    stringstream ss;
    ss << n;
    return ss.str();
}

ros::Publisher pub;

__u8* SpiWriteRead(int length, const char *device, uint8_t *tx)
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

    __u8 rx[length];
    for (int i = 0; i < length; i++) {
        tx[i] = 0x00;
        rx[i] = 0x00;
    }

    printf("tx: ");
    for(int i;i<Numel(tx);i++){
        cout << convert_int(tx[i]) << " ";
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
        cout << convert_int(rx[i]) << " ";
    }
    printf("\n");
// Close device
    close(fd);
}

void messageReceived(const my_pkg::Command& input) {
    ROS_INFO_STREAM(std::setprecision(2) << std::fixed << "upper=" << convert_int(input.upper) << "lower=" << convert_int(input.lower));

    my_pkg::Command output;
    output = myfunction(input);

    pub.publish(output);

    ROS_INFO_STREAM("Sending modified value:" << " upper=" <<  convert_int(output.upper) << " lower=" << convert_int(output.lower));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "spiInterface");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("my_topic0", 1000, &messageReceived);
    pub = nh.advertise<my_pkg::Command>("my_topic1", 1000);

    ros::spin();
}

my_pkg::Command myfunction(my_pkg::Command input) {
    my_pkg::Command output;
    unsigned char rightMsg[] = {input.upper, input.lower};
    int length = 2;
    const char *dev0 = "/dev/spidev1.0";
    __u8 rx[2];
    rx = SpiWriteRead(length, dev0, rightMsg);
    output.upper = rx[0];
    output.lower = rx[1];
    return output;
}