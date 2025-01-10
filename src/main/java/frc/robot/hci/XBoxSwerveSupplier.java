package frc.robot.hci;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class XBoxSwerveSupplier implements SwerveSupplier {

    private final CommandXboxController controller;

    public XBoxSwerveSupplier(CommandXboxController controller) {
        this.controller = controller;
    }

    @Override
    public LinearVelocity supplyX() {
        return MetersPerSecond.of(-controller.getLeftY()).times(MaxSpeed.in(MetersPerSecond));
    }

    @Override
    public LinearVelocity supplyY() {
        return MetersPerSecond.of(-controller.getLeftX()).times(MaxSpeed.in(MetersPerSecond));
    }

    @Override
    public AngularVelocity supplyRotationalRate() {
        if (!RobotBase.isSimulation()) {
            return RadiansPerSecond.of(-controller.getRightX()).times(MaxAngularRate.in(RadiansPerSecond));
        } else {
            return RadiansPerSecond.of(-controller.getRawAxis(2)).times(MaxAngularRate.in(RadiansPerSecond));
        }
    }
}
