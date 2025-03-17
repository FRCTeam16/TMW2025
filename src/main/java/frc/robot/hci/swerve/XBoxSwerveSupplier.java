package frc.robot.hci.swerve;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.util.GameInfo;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class XBoxSwerveSupplier implements SwerveSupplier {

    private final CommandXboxController controller;


    public XBoxSwerveSupplier(CommandXboxController controller) {
        this.controller = controller;
    }

    @Override
    public LinearVelocity supplyX() {
        double base = -controller.getLeftY();
        return MetersPerSecond.of(base).times(Constants.MaxSpeed.in(MetersPerSecond));
    }

    @Override
    public LinearVelocity supplyY() {
        double base = -controller.getLeftX();
        return MetersPerSecond.of(base).times(Constants.MaxSpeed.in(MetersPerSecond));
    }

    @Override
    public AngularVelocity supplyRotationalRate() {
        if (!RobotBase.isSimulation()) {
            return RadiansPerSecond.of(-controller.getRightX()).times(Constants.MaxAngularRate.in(RadiansPerSecond));
        } else {
            return RadiansPerSecond.of(-controller.getRawAxis(2)).times(Constants.MaxAngularRate.in(RadiansPerSecond));
        }
    }

    @Override
    public void setTargetHeading(Angle angle) {
    }

    @Override
    public void clearTargetHeading() {
    }
}
