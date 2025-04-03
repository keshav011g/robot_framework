#include <iostream>
#include <gpiod.h>
#include <unistd.h>
#include <cmath> // For angle calculations

// Servo GPIO pin (BCM numbering)
#define SERVO_PIN 18

// Servo parameters (adjust these based on your servo)
#define PWM_FREQUENCY 50 // Hz (standard for servos)
#define MIN_PULSE_WIDTH 500 // microseconds (0 degrees)
#define MAX_PULSE_WIDTH 2500 // microseconds (180 degrees)

// Function to set the servo angle
void setServoAngle(struct gpiod_line *line, int angle) {
    if (angle < 0 || angle > 180) {
        std::cerr << "Invalid angle. Angle must be between 0 and 180 degrees." << std::endl;
        return;
    }

    // Calculate pulse width
    double pulseWidth = MIN_PULSE_WIDTH + (angle / 180.0) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH);

    // Calculate on and off times in microseconds
    double onTime = pulseWidth;
    double offTime = (1000000.0 / PWM_FREQUENCY) - pulseWidth; //1/frequency = period in seconds, times 1000000 to get microseconds.

    // Set high and low states with delays
    gpiod_line_set_value(line, 1);
    usleep(onTime);
    gpiod_line_set_value(line, 0);
    usleep(offTime);

}

int main() {
    struct gpiod_chip *chip;
    struct gpiod_line *servoLine;
    int angle;

    // Open GPIO chip
    chip = gpiod_chip_open_by_name("gpiochip0");
    if (!chip) {
        std::cerr << "Error opening GPIO chip." << std::endl;
        return 1;
    }

    // Open servo GPIO line
    servoLine = gpiod_chip_get_line(chip, SERVO_PIN);
    if (!servoLine) {
        std::cerr << "Error getting GPIO line." << std::endl;
        gpiod_chip_close(chip);
        return 1;
    }

    // Configure servo line as output
    if (gpiod_line_request_output(servoLine, "servo", 0) != 0) {
        std::cerr << "Error requesting GPIO line as output." << std::endl;
        gpiod_chip_close(chip);
        return 1;
    }

    while (true) {
        std::cout << "Enter servo angle (0-180, or -1 to exit): ";
        std::cin >> angle;

        if (angle == -1) {
            break;
        }

        setServoAngle(servoLine, angle);
        // Add a small delay between commands
        usleep(20000); // 20ms
    }

    // Release resources
    gpiod_line_release(servoLine);
    gpiod_chip_close(chip);

    return 0;
}
