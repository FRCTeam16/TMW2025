package frc.robot.hci.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.Supplier;

public abstract class ControlBinding {
    protected final Joystick driveStick;
    protected final Joystick steerStick;
    protected final CommandXboxController joystick;

    public ControlBinding(Joystick driveStick, Joystick steerStick, CommandXboxController joystick) {
        this.driveStick = driveStick;
        this.steerStick = steerStick;
        this.joystick = joystick;
    }

    public abstract void bindControls();

    double deadband(double value, double deadband) {
        return Math.abs(value) < deadband ? 0 : value;
    }

    Supplier<Double> deadband(Supplier<Double> value, double deadband) {
        return () -> deadband(value.get(), deadband);
    }

    Trigger thresholdTrigger(Supplier<Double> value, double threshold) {
        return new Trigger(() -> Math.abs(value.get()) >= threshold);
    }
}
