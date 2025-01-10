package frc.robot.hci;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.RotationsPerSecond;

public interface SwerveSupplier {
    LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    AngularVelocity MaxAngularRate = RotationsPerSecond.of(0.75); // 3/4 of a rotation per second max angular velocity

    LinearVelocity supplyX();
    LinearVelocity supplyY();
    AngularVelocity supplyRotationalRate();
}
