package frc.robot.hci.swerve;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public interface SwerveSupplier {

    LinearVelocity supplyX();
    LinearVelocity supplyY();
    AngularVelocity supplyRotationalRate();
}
