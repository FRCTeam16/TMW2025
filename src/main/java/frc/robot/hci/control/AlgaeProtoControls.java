package frc.robot.hci.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems;

public class AlgaeProtoControls extends ControlBinding {

    public AlgaeProtoControls(Joystick driveStick, Joystick steerStick, CommandXboxController joystick) {
        super(driveStick, steerStick, joystick);
    }

    @Override
    public void bindControls() {
        joystick.y().onTrue(Subsystems.algaeIntake.intakeCommand()).onFalse(Subsystems.algaeIntake.holdAlgaeCommand());
        joystick.x().onTrue(Subsystems.algaeIntake.ejectCommand()).onFalse(Subsystems.algaeIntake.stopCommand());
        //                joystick.a().onTrue(Subsystems.algaeArm.setArmPositionCommand(AlgaeArm.AlgaeArmPosition.ReefLow));
//                joystick.y().onTrue(Subsystems.algaeArm.setArmPositionCommand(AlgaeArm.AlgaeArmPosition.ReefHigh));
    }
}