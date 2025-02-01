package frc.robot;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.RotationsPerSecond;

public class Constants {

    public static final LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    public static final AngularVelocity MaxAngularRate = RotationsPerSecond.of(1.0); // 3/4 of a rotation per second max angular velocity

    public enum JoystickMode {
        JoshPrototype,
        AustinGearboxPrototype,
        AlignmentTest,
        ElevatorProto
    }
    public static final boolean JoshPrototype = false;
    public static final boolean austinGearboxPrototype = true;
}
