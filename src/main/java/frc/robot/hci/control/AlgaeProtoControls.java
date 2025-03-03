package frc.robot.hci.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems;
import frc.robot.subsystems.Intake.AlgaeArm;

public class AlgaeProtoControls extends ControlBinding {

    public AlgaeProtoControls(Joystick driveStick, Joystick steerStick, CommandXboxController joystick) {
        super(driveStick, steerStick, joystick);
    }

    @Override
    public void bindControls() {
        joystick.y().onTrue(Subsystems.algaeIntake.intakeCommand()).onFalse(Subsystems.algaeIntake.holdAlgaeCommand());
        joystick.x().onTrue(Subsystems.algaeIntake.ejectCommand()).onFalse(Subsystems.algaeIntake.stopCommand());

        joystick.povUp().onTrue(Subsystems.algaeArm.setArmPositionCommand(AlgaeArm.AlgaeArmPosition.Start));
        joystick.povLeft().onTrue(Subsystems.algaeArm.setArmPositionCommand(AlgaeArm.AlgaeArmPosition.ReefHigh));
//        joystick.povRight().onTrue()
        joystick.povDown().onTrue(Subsystems.algaeArm.setArmPositionCommand(AlgaeArm.AlgaeArmPosition.Processor));
        //                joystick.a().onTrue(Subsystems.algaeArm.setArmPositionCommand(AlgaeArm.AlgaeArmPosition.ReefLow));
//                joystick.y().onTrue(Subsystems.algaeArm.setArmPositionCommand(AlgaeArm.AlgaeArmPosition.ReefHigh));
    }
}