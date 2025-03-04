package frc.robot.hci.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Subsystems;
import frc.robot.subsystems.Elevator;

public class CompBotDevControls extends ControlBinding {
    private final JoystickButton intakeCoral = new JoystickButton(driveStick, 2);
    private final JoystickButton shootCoral = new JoystickButton(driveStick, 1);

    private final JoystickButton intakeAlgae = new JoystickButton(steerStick, 2);
    private final JoystickButton shootAlgae = new JoystickButton(steerStick, 1);

    private final POVButton elevatorDown = new POVButton(driveStick, 180);
    private final POVButton elevatorL2 = new POVButton(driveStick, 270);
    private final POVButton elevatorL3 = new POVButton(driveStick, 90);
    private final POVButton elevatorL4 = new POVButton(driveStick, 0);


    public CompBotDevControls(Joystick driveStick, Joystick steerStick, CommandXboxController joystick) {
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
        elevatorL2.onTrue(new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L2));
        elevatorL3.onTrue(new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L3));
        elevatorL4.onTrue(new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L4));
    }
}
