package frc.robot.hci.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems;

public class CoralTestingControls extends ControlBinding {
    public CoralTestingControls(Joystick driveStick, Joystick steerStick, CommandXboxController joystick) {
        super(driveStick, steerStick, joystick);
    }

    @Override
    public void bindControls() {
        joystick.povLeft().and(joystick.a()).onTrue(Subsystems.coralIntake.intakeCoralCommand());
        joystick.povLeft().and(joystick.b()).onTrue(Subsystems.coralIntake.stopCommand());
        joystick.povLeft().and(joystick.y()).whileTrue(Subsystems.coralIntake.ejectCommand());
        joystick.povLeft().and(joystick.x()).whileTrue(Subsystems.coralIntake.shootCoralCommand());
    }
}
