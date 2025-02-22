package frc.robot.hci.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.commands.PathfindToScoringPositionCommand;
import frc.robot.subsystems.Elevator;

import java.util.function.Supplier;

public class ScrimmageControls extends ControlBinding {
    private final JoystickButton intakeCoral = new JoystickButton(driveStick, 2);
    private final JoystickButton shootCoral = new JoystickButton(driveStick, 1);
    private final JoystickButton alignLeft = new JoystickButton(driveStick, 3);
    private final JoystickButton alignRight = new JoystickButton(driveStick, 4);

    private final JoystickButton intakeAlgae = new JoystickButton(steerStick, 2);
    private final JoystickButton shootAlgae = new JoystickButton(steerStick, 1);

    private final Trigger startConveyor = joystick.leftBumper();
    private final Trigger stopConveyor = joystick.rightBumper();


    private final Trigger elevatorDown = joystick.rightTrigger();
    private final Trigger elevatorL1 = joystick.x();
    private final Trigger elevatorL2 = joystick.a();
    private final Trigger elevatorL3 = joystick.b();
    private final Trigger elevatorL4 = joystick.y();


    private final Trigger manualAlgaeToggleButton = joystick.leftStick();
    private final Supplier<Double> manualAlgaeArmControl = deadband(joystick::getLeftY, 0.05);
    private final Trigger manualElevatorToggleButton = joystick.rightStick();
    private final Supplier<Double> manualElevatorControl = Robot.isReal() ?
            deadband(joystick::getRightY, 0.05) :
            deadband(joystick::getRightTriggerAxis, 0.05);  // simulation mode is flipped


    public ScrimmageControls(Joystick driveStick, Joystick steerStick, CommandXboxController joystick) {
        super(driveStick, steerStick, joystick);
    }

    @Override
    public void bindControls() {
        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(Subsystems.swerveSubsystem.runOnce(Subsystems.swerveSubsystem::seedFieldCentric));

        intakeCoral.whileTrue(Subsystems.coralIntake.intakeCoralCommand());
        shootCoral.whileTrue(Subsystems.coralIntake.shootCoralCommand());

        intakeAlgae.onTrue(Subsystems.algaeIntake.intakeCommand()).onFalse(Subsystems.algaeIntake.holdAlgaeCommand());
        shootAlgae.onTrue(Subsystems.algaeIntake.ejectCommand()).onFalse(Subsystems.algaeIntake.stopCommand());

        elevatorDown.onTrue(new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.Zero));
        elevatorL1.onTrue(new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.TROUGH));
        elevatorL2.onTrue(new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L2));
        elevatorL3.onTrue(new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L3));
        elevatorL4.onTrue(new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L4));

        manualAlgaeToggleButton.toggleOnTrue(Subsystems.algaeArm.openLoopCommand(manualAlgaeArmControl));
        manualElevatorToggleButton.toggleOnTrue(Subsystems.elevator.openLoopCommand(manualElevatorControl));


        alignLeft.whileTrue(new PathfindToScoringPositionCommand(true));
        alignRight.whileTrue(new PathfindToScoringPositionCommand(false));
//        alignLeft.whileTrue(new LimelightBasedAlignmentCommand(LimelightBasedAlignmentCommand.TargetSide.LEFT));
//        alignRight.whileTrue(new LimelightBasedAlignmentCommand(LimelightBasedAlignmentCommand.TargetSide.RIGHT));

        startConveyor.whileTrue(Subsystems.funnelSubsystem.startConveyorCommand());
        stopConveyor.whileTrue(Subsystems.funnelSubsystem.stopConveyorCommand());

        bindDebugControls();
    }

    void bindDebugControls() {
        SmartDashboard.putData("Move Funnel to 5", Subsystems.funnelSubsystem.movePivotToPosition(5));
        SmartDashboard.putData("Move Funnel to 0", Subsystems.funnelSubsystem.movePivotToPosition(0));

    }
}
