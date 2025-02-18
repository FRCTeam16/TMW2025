package frc.robot.hci.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlignmentTest;

public class AlignmentTestControls extends ControlBinding {

    public AlignmentTestControls(Joystick driveStick, Joystick steerStick, CommandXboxController joystick) {
        super(driveStick, steerStick, joystick);
    }

    @Override
    public void bindControls() {
        joystick.x().whileTrue(new AlignmentTest(AlignmentTest.TargetSide.LEFT));
        joystick.b().whileTrue(new AlignmentTest(AlignmentTest.TargetSide.RIGHT));
    }
}