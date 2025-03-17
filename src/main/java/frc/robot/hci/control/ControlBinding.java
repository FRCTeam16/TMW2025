package frc.robot.hci.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.hci.swerve.JoystickSwerveSupplier;
import frc.robot.hci.swerve.SwerveSupplier;
import frc.robot.hci.swerve.XBoxSwerveSupplier;
import frc.robot.subsystems.Lifecycle;

import java.util.function.Supplier;

/**
 * Abstract class for binding controls to commands
 */
public abstract class ControlBinding implements Lifecycle {
    protected final Joystick driveStick;
    protected final Joystick steerStick;
    protected final CommandXboxController joystick;
    protected final SwerveSupplier swerveSupplier;

    public ControlBinding(Joystick driveStick, Joystick steerStick, CommandXboxController joystick) {
        this.driveStick = driveStick;
        this.steerStick = steerStick;
        this.joystick = joystick;

        this.swerveSupplier = (!RobotBase.isSimulation()) ?
                new JoystickSwerveSupplier(driveStick, steerStick, joystick) :
                new XBoxSwerveSupplier(joystick);

    }

    public SwerveSupplier getSwerveSupplier() {
       return this.swerveSupplier;
    }

    /**
     * Bind the controls to commands
     */
    public abstract void bindControls();


    /**
     * Periodic method to be overridden by subclasses who need to update their state or display information
     */
    public void periodic() {}

    /**
     * Apply a deadband to a value
     * @param value
     * @param deadband
     * @return the deadbanded value
     */
    double deadband(double value, double deadband) {
        return Math.abs(value) < deadband ? 0 : value;
    }

    /**
     * Apply a deadband to a value
     * @param value
     * @param deadband
     * @return Supplier of the deadbanded value
     */
    Supplier<Double> deadband(Supplier<Double> value, double deadband) {
        return () -> deadband(value.get(), deadband);
    }

    /**
     * Create a trigger for a button
     * @param value
     * @param threshold
     * @return
     */
    Trigger thresholdTrigger(Supplier<Double> value, double threshold) {
        return new Trigger(() -> Math.abs(value.get()) >= threshold);
    }
}
