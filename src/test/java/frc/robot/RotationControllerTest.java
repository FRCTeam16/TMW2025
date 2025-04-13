package frc.robot;

import frc.robot.subsystems.RotationController;
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class RotationControllerTest {

    @Test
    void testPIDControllerWithRadians() {
        // Initialize the PID controller with gains converted for radians
        double p = 0.016 * (180 / Math.PI);
        double i = 0.0 * (180 / Math.PI);
        double d = 0.0 * (180 / Math.PI);
        RotationController rotationPIDRadians = new RotationController(p, i, d);

        // Define test cases
        double setpoint = Math.PI / 2; // 90 degrees in radians
        double measurement = Math.PI / 4; // 45 degrees in radians
        double expectedOutput = p * (setpoint - measurement); // Proportional term only

        // Calculate the output
        double output = rotationPIDRadians.calculate(measurement, setpoint);

        // Assert the output is as expected
        assertEquals(expectedOutput, output, 1e-5, "PID output should match expected value");
    }
}