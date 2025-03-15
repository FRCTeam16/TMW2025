package frc.robot.hci.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.commands.*;
import frc.robot.commands.dms.CoralIntakeAMDCommand;
import frc.robot.commands.dms.RunDMSCommand;
import frc.robot.commands.pose.GenericPoseRequestCommand;
import frc.robot.commands.vision.AlignDriveInCommand;
import frc.robot.commands.vision.AlignDriveInCommandAlgea;
import frc.robot.commands.vision.PipelineSwitcher;
import frc.robot.commands.vision.VisionPoseUpdateFactory;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake.AlgaeArm;
import frc.robot.subsystems.Intake.AlgaeArm.AlgaeArmPosition;
import frc.robot.subsystems.pose.ResetToAlliancePoseRequest;
import frc.robot.subsystems.pose.SeedFieldCentricRequest;
import frc.robot.subsystems.vision.Pipeline;
import frc.robot.util.BSMath;

import java.util.function.Supplier;

public class BayouControls extends ControlBinding{
    final JoystickButton shootCoral = new JoystickButton(driveStick, 1);
    final JoystickButton alignLeft = new JoystickButton(driveStick, 3);
    final JoystickButton alignRight = new JoystickButton(driveStick, 4);

    final JoystickButton alignMiddle = new JoystickButton(steerStick, 3);

    final JoystickButton intakeAlgae = new JoystickButton(steerStick, 2);
    final JoystickButton shootAlgae = new JoystickButton(steerStick, 1);
//    final JoystickButton robotCentric = new JoystickButton(steerStick, 3);


    final Trigger elevatorDown = joystick.rightTrigger();
    final Trigger elevatorL1 = joystick.x();
    final Trigger elevatorL2 = joystick.a();
    final Trigger elevatorL3 = joystick.b();
    final Trigger elevatorL4 = joystick.y();

    final Trigger intakeCoral = joystick.leftTrigger();

    final Trigger algaeArmFloor = joystick.povRight();
    final Trigger algaeArmProcessor = joystick.povLeft();
    //TODO: "AUTOMATE"?
    final Trigger algaeArmUp = joystick.leftStick();
    final Trigger algaeHighElevator = joystick.rightBumper();
    final Trigger algaeLowElevator = joystick.leftBumper();

    final Trigger enableClimb = joystick.start();
    // final Trigger climberUp = joystick.povLeft();
    // final Trigger climberDown = joystick.povRight();
    final Trigger climberPickup = joystick.povDown();
    final Trigger climberClimb = joystick.povUp();

    final Trigger resetPose = joystick.back();

    private final Trigger manualAlgaeToggleButton = joystick.leftStick();
    private final Supplier<Double> manualAlgaeArmControl = algaeArmDampener(); // deadband(joystick::getLeftY, 0.05);
    private final Trigger manualElevatorToggleButton = joystick.rightStick();
    private final Supplier<Double> manualElevatorControl = Robot.isReal() ?
            deadband(joystick::getRightY, 0.05) :
            deadband(joystick::getRightTriggerAxis, 0.05);  // simulation mode is flipped


    public BayouControls(Joystick driveStick, Joystick steerStick, CommandXboxController joystick) {
        super(driveStick, steerStick, joystick);
    }

    @Override
    public void bindControls() {

        intakeCoral.whileTrue(Subsystems.coralIntake.intakeCoralCommand());
        shootCoral.whileTrue(Subsystems.coralIntake.shootCoralCommand());

        intakeAlgae.onTrue(Subsystems.algaeIntake.intakeCommand()).onFalse(Subsystems.algaeIntake.holdAlgaeCommand());
        shootAlgae.onTrue(Subsystems.algaeIntake.ejectCommand()).onFalse(Subsystems.algaeIntake.stopCommand());

        elevatorDown.onTrue(new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.Zero));
        elevatorL1.onTrue(new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.TROUGH));
        elevatorL2.onTrue(new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L2));
        elevatorL3.onTrue(new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L3));
        elevatorL4.onTrue(new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L4));

        algaeArmFloor.onTrue(new AlgaeArm.SetArmPositionCommand(AlgaeArmPosition.Ground));
        algaeArmProcessor.onTrue(new AlgaeArm.SetArmPositionCommand(AlgaeArmPosition.Processor));
        algaeArmUp.onTrue(new AlgaeArm.SetArmPositionCommand(AlgaeArmPosition.Up));

        algaeHighElevator.onTrue(new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.AlgaeReefHigh));
        algaeLowElevator.onTrue(new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.AlgaeReefLow));

        // climberUp.onTrue(new Climber.ClimberMoveToPositionCommand(Climber.ClimberPosition.UP));
        // climberDown.onTrue(new Climber.ClimberMoveToPositionCommand(Climber.ClimberPosition.DOWN));
        enableClimb.and(climberPickup).onTrue(new Climber.ClimberMoveToPositionCommand(Climber.ClimberPosition.PICKUP));
        enableClimb.and(climberClimb).onTrue(new Climber.ClimberMoveToPositionCommand(Climber.ClimberPosition.CLIMB));


        manualAlgaeToggleButton.toggleOnTrue(Subsystems.algaeArm.openLoopCommand(manualAlgaeArmControl));
        enableClimb.and(manualElevatorToggleButton).toggleOnTrue(Subsystems.elevator.openLoopCommand(manualElevatorControl));

//        alignLeft.whileTrue(PathfindFactory.limelightAlignToVisibleAprilTag(true));
//        alignRight.whileTrue(PathfindFactory.limelightAlignToVisibleAprilTag(false));
        alignLeft.whileTrue(new AlignDriveInCommand(true)); // SimpleAlignCommand
        alignRight.whileTrue(new AlignDriveInCommand(false));

        alignMiddle.whileTrue(new AlignDriveInCommandAlgea());

//        robotCentric.whileTrue(new DriveRobotCentricCommand());

    }

    Supplier<Double> algaeArmDampener() {
        Supplier<Double> dampener = () ->
                BSMath.map(joystick.getLeftY(), -1, 1, -0.1, 0.1);
        return deadband(dampener, 0.05);
    }
    
}