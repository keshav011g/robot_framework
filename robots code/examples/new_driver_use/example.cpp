#include "SparkFunLIS3DH.h" // Assuming your header file is named SparkFunLIS3DH.h
#include <iostream>
#include <unistd.h> // For usleep

int main() {
    // I2C address of LIS3DH (check your sensor's datasheet)
    uint8_t lis3dhAddress = 0x18; // Example address, adjust if needed

    // Create an LIS3DH object
    LIS3DH lis3dh(lis3dhAddress);

    // Initialize the sensor
    if (lis3dh.begin() != IMU_SUCCESS) {
        std::cerr << "Failed to initialize LIS3DH sensor." << std::endl;
        return 1;
    }

    std::cout << "LIS3DH sensor initialized successfully." << std::endl;

    // Read accelerometer data in a loop
    for (int i = 0; i < 10; ++i) {
        float x = lis3dh.readFloatAccelX();
        float y = lis3dh.readFloatAccelY();
        float z = lis3dh.readFloatAccelZ();

        std::cout << "Accel X: " << x << " g, Y: " << y << " g, Z: " << z << " g" << std::endl;

        usleep(100000); // Wait 100 milliseconds (adjust as needed)
    }

    // Example of changing settings after initialization.
    lis3dh.settings.accelRange = 8; // Change the range to +-8g.
    lis3dh.applySettings(); //Apply the changed settings.

    std::cout << "Range changed to +-8g" << std::endl;

    for (int i = 0; i < 10; ++i) {
        float x = lis3dh.readFloatAccelX();
        float y = lis3dh.readFloatAccelY();
        float z = lis3dh.readFloatAccelZ();

        std::cout << "Accel X: " << x << " g, Y: " << y << " g, Z: " << z << " g" << std::endl;

        usleep(100000); // Wait 100 milliseconds (adjust as needed)
    }

    return 0;
}
