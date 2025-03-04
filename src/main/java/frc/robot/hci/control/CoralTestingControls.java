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
        (joystick.a()).whileTrue(Subsystems.coralIntake.intakeCoralCommand());
        (joystick.b()).onTrue(Subsystems.coralIntake.stopCommand());
        (joystick.y()).whileTrue(Subsystems.coralIntake.ejectCommand());
        (joystick.x()).whileTrue(Subsystems.coralIntake.shootCoralCommand());
    }
}
