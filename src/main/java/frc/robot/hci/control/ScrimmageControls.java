package frc.robot.hci.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems;
import frc.robot.subsystems.Elevator;

public class ScrimmageControls extends ControlBinding {
    private final JoystickButton intakeCoral = new JoystickButton(driveStick, 2);
    private final JoystickButton shootCoral = new JoystickButton(driveStick, 1);

    private final JoystickButton intakeAlgae = new JoystickButton(steerStick, 2);
    private final JoystickButton shootAlgae = new JoystickButton(steerStick, 1);

    private final Trigger elevatorDown = joystick.rightTrigger();
    private final Trigger elevatorL1 = joystick.x();
    private final Trigger elevatorL2 = joystick.a();
    private final Trigger elevatorL3 = joystick.b();
    private final Trigger elevatorL4 = joystick.y();


    public ScrimmageControls(Joystick driveStick, Joystick steerStick, CommandXboxController joystick) {
        super(driveStick, steerStick, joystick);
    }

    @Override
    public void bindControls() {
        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(Subsystems.swerveSubsystem.runOnce(Subsystems.swerveSubsystem::seedFieldCentric));

        intakeCoral.onTrue(Subsystems.coralIntake.intakeCoralCommand());
        shootCoral.whileTrue(Subsystems.coralIntake.shootCoralCommand());

        intakeAlgae.onTrue(Subsystems.algaeIntake.intakeCommand()).onFalse(Subsystems.algaeIntake.holdAlgaeCommand());
        shootAlgae.onTrue(Subsystems.algaeIntake.ejectCommand()).onFalse(Subsystems.algaeIntake.stopCommand());

        elevatorDown.onTrue(new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.Zero));
        elevatorL1.onTrue(new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.Zero));
        elevatorL2.onTrue(new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L2));
        elevatorL3.onTrue(new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L3));
        elevatorL4.onTrue(new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L4));
    }
}
