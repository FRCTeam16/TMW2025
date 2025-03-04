package frc.robot.hci.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems;

public class ElevatorProtoControls extends ControlBinding {

    public ElevatorProtoControls(Joystick driveStick, Joystick steerStick, CommandXboxController joystick) {
        super(driveStick, steerStick, joystick);
    }

    @Override
    public void bindControls() {
        joystick.a().onTrue(Subsystems.elevator.openLoopDownCommand()).onFalse(Subsystems.elevator.openLoopStopCommand());
        joystick.y().onTrue(Subsystems.elevator.openLoopUpCommand()).onFalse(Subsystems.elevator.openLoopStopCommand());
    }
}