package frc.robot.hci.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.subsystems.RotationController;
import frc.robot.util.BSLogger;
import frc.robot.util.GameInfo;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public class JoystickSwerveSupplier implements SwerveSupplier {
    private final Joystick steerStick;
    private final Joystick driveStick;
    private final CommandXboxController controller;

    private final RotationController rotationPID = new RotationController(0.016, 0, 0);


    @SuppressWarnings("OptionalUsedAsFieldOrParameterType")
    private Optional<Angle> targetHeading = Optional.empty();

    public JoystickSwerveSupplier(Joystick driveStick, Joystick steerStick, CommandXboxController controller) {
        this.steerStick = steerStick;
        this.driveStick = driveStick;
        this.controller = controller;

        rotationPID.setTolerance(1.0);

        SmartDashboard.setDefaultNumber("Controls/ElevGovSpeed", 0.4);
        SmartDashboard.putData("Controls/JoystickAlignPID", rotationPID);
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
            return value * SmartDashboard.getNumber("Controls/ElevGovSpeed", 0.4);
        } else {
            return value;
        }
    }

    protected double getBaseX() {
        return applyLimiter(applyDeadband(
                -driveStick.getY(), 0.08) );
    }

    protected double getBaseY() {
        return applyLimiter(applyDeadband(
                -driveStick.getX(), 0.05));
    }

    protected double getBaseRotationalRate() {
        final double baseRotation;
        if (targetHeading.isEmpty()) {
            baseRotation = applyDeadband(-steerStick.getX(), 0.1);
        } else {
            double robotDegrees = Subsystems.swerveSubsystem.getState().Pose.getRotation().getDegrees();
            baseRotation = rotationPID.calculate(robotDegrees, targetHeading.get().in(Degrees));
        }
        return applyLimiter(baseRotation);
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
        double base = MathUtil.clamp(getBaseRotationalRate(), -1.0, 1.0);
        // return RadiansPerSecond.of(base).times(Constants.MaxAngularRate.in(RadiansPerSecond));
        return DegreesPerSecond.of(320).times(base);
    }

    @Override
    public void setTargetHeading(Angle angle) {
        BSLogger.log("JoystickSwerveSupplier", "setTargetHeading: " + angle);
        rotationPID.reset();
        this.targetHeading = Optional.of(angle);
    }

    @Override
    public void clearTargetHeading() {
        BSLogger.log("JoystickSwerveSupplier", "clearTargetHeading");
        this.targetHeading = Optional.empty();
    }
}