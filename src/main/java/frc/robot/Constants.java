package frc.robot;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class Constants {

    public static final LinearVelocity MaxSpeed = MetersPerSecond.of(4.95);
    public static final AngularVelocity MaxAngularRate = RotationsPerSecond.of(1.0);

    public enum JoystickMode {
        JoshPrototype,
        AustinGearboxPrototype,
        AlignmentTest,
        ElevatorProto,
        climberProto,
        AlgaeProto,
    }
    public static final boolean JoshPrototype = false;
    public static final boolean austinGearboxPrototype = false;
}
