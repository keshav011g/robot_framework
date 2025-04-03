#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

int main() {
    int file;
    int address = 0x68; // Replace with your I2C device address
    char filename[20];
    char buf[2];

    sprintf(filename, "/dev/i2c-1"); // Use /dev/i2c-0 for older Pis

    if ((file = open(filename, O_RDWR)) < 0) {
        std::cerr << "Failed to open I2C bus\n";
        return 1;
    }

    if (ioctl(file, I2C_SLAVE, address) < 0) {
        std::cerr << "Failed to acquire bus access and/or talk to slave.\n";
        close(file);
        return 1;
    }

    // Example: Writing data to the I2C device
    buf[0] = 0x00; // Register address to write to
    buf[1] = 0x42; // Data to write
    if (write(file, buf, 2) != 2) {
        std::cerr << "Failed to write to the I2C bus.\n";
    }

    // Example: Reading data from the I2C device
    buf[0] = 0x00; // Register address to read from
    if (write(file, buf, 1) != 1) {
        std::cerr << "Failed to write register address.\n";
    }

    if (read(file, buf, 1) != 1) {
        std::cerr << "Failed to read from the I2C bus.\n";
    } else {
        std::cout << "Read value: " << (int)buf[0] << std::endl;
    }

    close(file);
    return 0;
}
}
