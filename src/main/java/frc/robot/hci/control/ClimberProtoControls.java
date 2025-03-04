package frc.robot.hci.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems;

public class ClimberProtoControls extends ControlBinding {

    public ClimberProtoControls(Joystick driveStick, Joystick steerStick, CommandXboxController joystick) {
        super(driveStick, steerStick, joystick);
    }

    @Override
    public void bindControls() {
        joystick.rightBumper().and(joystick.a()).onTrue(Subsystems.Climberproto1.runForward()).onFalse(Subsystems.Climberproto1.stop());
        joystick.rightBumper().and(joystick.y()).onTrue(Subsystems.Climberproto1.runBackward()).onFalse(Subsystems.Climberproto1.stop());
        joystick.leftBumper().and(joystick.a()).onTrue(Subsystems.Climberproto2.runForward()).onFalse(Subsystems.Climberproto2.stop());
        joystick.leftBumper().and(joystick.y()).onTrue(Subsystems.Climberproto2.runBackward()).onFalse(Subsystems.Climberproto2.stop());
        joystick.b().onTrue(Subsystems.Climberproto1.stop());
        joystick.b().onTrue(Subsystems.Climberproto2.stop());
    }
}