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
import frc.robot.commands.pose.GenericPoseRequestCommand;
import frc.robot.commands.vision.AlignDriveInCommand;
import frc.robot.commands.vision.PipelineSwitcher;
import frc.robot.commands.vision.VisionPoseUpdateFactory;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.pose.ResetToAlliancePoseRequest;
import frc.robot.subsystems.pose.SeedFieldCentricRequest;
import frc.robot.subsystems.vision.Pipeline;
import frc.robot.util.BSMath;

import java.util.function.Supplier;

public class ScrimmageControls extends ControlBinding {
    private final JoystickButton intakeCoral = new JoystickButton(driveStick, 2);
    private final JoystickButton shootCoral = new JoystickButton(driveStick, 1);
    private final JoystickButton alignLeft = new JoystickButton(driveStick, 3);
    private final JoystickButton alignRight = new JoystickButton(driveStick, 4);

    private final JoystickButton intakeAlgae = new JoystickButton(steerStick, 2);
    private final JoystickButton shootAlgae = new JoystickButton(steerStick, 1);
//    private final JoystickButton robotCentric = new JoystickButton(steerStick, 3);


    private final Trigger elevatorDown = joystick.rightTrigger();
    private final Trigger elevatorL1 = joystick.x();
    private final Trigger elevatorL2 = joystick.a();
    private final Trigger elevatorL3 = joystick.b();
    private final Trigger elevatorL4 = joystick.y();

    private final Trigger algaeHighElevator = joystick.povUp();
    private final Trigger algaeLowElevator = joystick.povDown();

    private final Trigger enableClimb = joystick.leftTrigger();
    private final Trigger climberUp = joystick.povLeft();
    private final Trigger climberDown = joystick.povRight();
    private final Trigger climberPickup = joystick.leftBumper();
    private final Trigger climberClimb = joystick.rightBumper();

    private final Trigger resetPose = joystick.back();

    private final Trigger manualAlgaeToggleButton = joystick.leftStick();
    private final Supplier<Double> manualAlgaeArmControl = algaeArmDampener(); // deadband(joystick::getLeftY, 0.05);
    private final Trigger manualElevatorToggleButton = joystick.rightStick();
    private final Supplier<Double> manualElevatorControl = Robot.isReal() ?
            deadband(joystick::getRightY, 0.05) :
            deadband(joystick::getRightTriggerAxis, 0.05);  // simulation mode is flipped


    public ScrimmageControls(Joystick driveStick, Joystick steerStick, CommandXboxController joystick) {
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

        algaeHighElevator.onTrue(new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.AlgaeReefHigh));
        algaeLowElevator.onTrue(new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.AlgaeReefLow));

        climberUp.onTrue(new Climber.ClimberMoveToPositionCommand(Climber.ClimberPosition.UP));
        climberDown.onTrue(new Climber.ClimberMoveToPositionCommand(Climber.ClimberPosition.DOWN));
        enableClimb.and(climberPickup).onTrue(new Climber.ClimberMoveToPositionCommand(Climber.ClimberPosition.PICKUP));
        enableClimb.and(climberClimb).onTrue(new Climber.ClimberMoveToPositionCommand(Climber.ClimberPosition.CLIMB));


        manualAlgaeToggleButton.toggleOnTrue(Subsystems.algaeArm.openLoopCommand(manualAlgaeArmControl));
        manualElevatorToggleButton.toggleOnTrue(Subsystems.elevator.openLoopCommand(manualElevatorControl));

//        alignLeft.whileTrue(PathfindFactory.limelightAlignToVisibleAprilTag(true));
//        alignRight.whileTrue(PathfindFactory.limelightAlignToVisibleAprilTag(false));
        alignLeft.whileTrue(new AlignDriveInCommand(true)); // SimpleAlignCommand
        alignRight.whileTrue(new AlignDriveInCommand(false));

//        robotCentric.whileTrue(new DriveRobotCentricCommand());

        resetPose.onTrue(
                VisionPoseUpdateFactory.resetFromMainPoseEstimator().ignoringDisable(true)
        );

        bindCommonButtons();
        bindDebugControls();
    }

    void bindCommonButtons() {
        SmartDashboard.putData("Reset Pose From Vision",
                VisionPoseUpdateFactory.resetFromMainPoseEstimator().ignoringDisable(true));

        SmartDashboard.putData("Reset Alliance Pose", new GenericPoseRequestCommand<>(ResetToAlliancePoseRequest.class));

        SmartDashboard.putData("Set LLs to Apriltag", new PipelineSwitcher(Pipeline.April));
        SmartDashboard.putData("Set LLs to Viewfinder", new PipelineSwitcher(Pipeline.View));

        // reset the field-centric heading on left bumper press
        SmartDashboard.putData("Seed Field Centric", Commands.runOnce(() -> Subsystems.poseManager.pushRequest(new SeedFieldCentricRequest()))
                .ignoringDisable(true));

        SmartDashboard.putData("Zero Climber", Subsystems.climber.zeroClimberPosition().ignoringDisable(true));
    }

    void bindDebugControls() {
        SmartDashboard.putData("TestTargetCalc", new TestTargetPoseCalc().ignoringDisable(true));

        SmartDashboard.putData("Open Latch", Subsystems.funnelSubsystem.openLatchCommand().ignoringDisable(true));
        SmartDashboard.putData("Close Latch", Subsystems.funnelSubsystem.closeLatchCommand().ignoringDisable(true));
    }

    Supplier<Double> algaeArmDampener() {
        Supplier<Double> dampener = () ->
                BSMath.map(joystick.getLeftY(), -1, 1, -0.1, 0.1);
        return deadband(dampener, 0.05);
    }
}
