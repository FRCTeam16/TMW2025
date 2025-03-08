package frc.robot.hci.swerve;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.util.GameInfo;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class JoystickSwerveSupplier implements SwerveSupplier {
    private final Joystick steerStick;
    private final Joystick driveStick;
    private final CommandXboxController controller;
    private final boolean isRedAlliance;   // TODO Centralized world state lookup?

    public JoystickSwerveSupplier(Joystick driveStick, Joystick steerStick, CommandXboxController controller) {
        this.steerStick = steerStick;
        this.driveStick = driveStick;
        this.controller = controller;
        this.isRedAlliance = GameInfo.isRedAlliance();

        SmartDashboard.setDefaultNumber("ElevGovSpeed", 0.4);
    }

    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            return value;
        } else {
            return 0.0;
        }
    }

    protected double applyLimiter(double value) {
        boolean isElevatorUp = Subsystems.elevator.isElevatorUp();
        if (isElevatorUp) {
            return value * SmartDashboard.getNumber("ElevGovSpeed", 0.4);
        } else {
            return value;
        }
    }

    protected double getBaseX() {
        return applyLimiter(applyDeadband(
                -driveStick.getY(), 0.08) * (isRedAlliance ? 1 : 1));
    }

    protected double getBaseY() {
        return applyLimiter(applyDeadband(
                -driveStick.getX(), 0.05) * (isRedAlliance ? 1 : 1));
    }

    protected double getBaseRotationalRate() {
        return applyLimiter(applyDeadband(
                -steerStick.getX(), 0.05) * (isRedAlliance ? 1 : 1));
    }


    @Override
    public LinearVelocity supplyX() {
        double base = getBaseX();
        return MetersPerSecond.of(base).times(Constants.MaxSpeed.in(MetersPerSecond));
    }

    @Override
    public LinearVelocity supplyY() {
        double base = getBaseY();
        return MetersPerSecond.of(base).times(Constants.MaxSpeed.in(MetersPerSecond));
    }

    @Override
    public AngularVelocity supplyRotationalRate() {
        double base = getBaseRotationalRate();
        return RadiansPerSecond.of(base).times(Constants.MaxAngularRate.in(RadiansPerSecond));
    }
}