package frc.robot.hci.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
}
