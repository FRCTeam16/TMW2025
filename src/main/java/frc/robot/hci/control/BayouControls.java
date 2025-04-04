package frc.robot.hci.control;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.commands.PickAlgaeCommand;
import frc.robot.commands.PickAlgaeSoonerCommand;
import frc.robot.commands.RotateToAngleCommand;
import frc.robot.commands.amd.*;
import frc.robot.commands.pose.GenericPoseRequestCommand;
import frc.robot.commands.vision.AlignDriveInCommand;
import frc.robot.commands.vision.PipelineSwitcher;
import frc.robot.commands.vision.VisionPoseUpdateFactory;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake.AlgaeArm;
import frc.robot.subsystems.Intake.AlgaeArm.AlgaeArmPosition;
import frc.robot.subsystems.amd.AMDSerialData;
import frc.robot.subsystems.pose.ResetToAlliancePoseRequest;
import frc.robot.subsystems.pose.SeedFieldCentricRequest;
import frc.robot.subsystems.vision.Pipeline;
import frc.robot.util.BSMath;
import frc.robot.util.GameInfo;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;

public class BayouControls extends ControlBinding {

    // Custom State for toggles
    private boolean climbEnabledState = false;
    private final Trigger climbEnabled = new Trigger(() -> climbEnabledState);

    private boolean manualStickControlState = false;
    private final Trigger manualStickControl = new Trigger(() -> manualStickControlState);


    //
    // Primary Controls
    //
    final JoystickButton shootCoral = new JoystickButton(driveStick, 1);
    final JoystickButton alignMiddle = new JoystickButton(driveStick, 2);
    final JoystickButton alignLeft = new JoystickButton(driveStick, 3);
    final JoystickButton alignRight = new JoystickButton(driveStick, 4);

    final JoystickButton shootAlgae = new JoystickButton(steerStick, 1);
    final JoystickButton intakeAlgae = new JoystickButton(steerStick, 2);
    final JoystickButton alignLeftStation = new JoystickButton(steerStick, 3);
    final JoystickButton alignRightStation = new JoystickButton(steerStick, 4);

    final JoystickButton overrideClimberUp = new JoystickButton(driveStick, 10);
    final JoystickButton overrideClimberDown = new JoystickButton(driveStick, 11);

    final JoystickButton resetPoseButton = new JoystickButton(driveStick, 14);


    final Trigger elevatorDown = joystick.rightTrigger();
    final Trigger elevatorL1 = joystick.x();
    final Trigger elevatorL2 = joystick.a();
    final Trigger elevatorL3 = joystick.b();
    final Trigger elevatorL4 = joystick.y();

    final Trigger intakeCoral = joystick.leftTrigger();

    final Trigger algaeArmFloor = joystick.povRight();
    final Trigger algaeArmProcessor = joystick.povLeft();
    final Trigger algaeArmUp = joystick.leftStick().and(() -> !manualStickControlState);
    final Trigger algaeHighElevator = joystick.rightBumper();
    final Trigger algaeLowElevator = joystick.leftBumper();

    final Trigger enableClimbToggle = joystick.start();
    final Trigger climberPickup = climbEnabled.and(joystick.povDown());
    final Trigger climberClimb = climbEnabled.and(joystick.povUp());

    final Trigger enableManualStickControlToggle = joystick.back();

    private final Supplier<Double> manualAlgaeArmControl = algaeArmDampener(); // deadband(joystick::getLeftY, 0.05);

    private final Supplier<Double> manualElevatorControl = Robot.isReal() ?
            deadband(joystick::getRightY, 0.05) :
            deadband(joystick::getRightTriggerAxis, 0.05);  // simulation mode is flipped

    // Physical buttons
    private final DigitalInput amdStartSignal = new DigitalInput(0);
    private final Trigger amdButton = new Trigger(() -> !amdStartSignal.get()).debounce(1.0);



    /**
     * Constructor for the BayouControls class
     * @param driveStick
     * @param steerStick
     * @param joystick
     */
    public BayouControls(Joystick driveStick, Joystick steerStick, CommandXboxController joystick) {
        super(driveStick, steerStick, joystick);

        SmartDashboard.setDefaultBoolean("Controls/Climb Enabled", climbEnabledState);
        SmartDashboard.setDefaultBoolean("Controls/Manual Stick Control", manualStickControlState);
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

        algaeArmFloor.whileTrue(new AlgaeArm.SetArmPositionCommand(AlgaeArmPosition.Ground))
                .onFalse(new AlgaeArm.SetArmPositionCommand(AlgaeArmPosition.Up));
        algaeArmProcessor.whileTrue(new AlgaeArm.SetArmPositionCommand(AlgaeArmPosition.Processor))
                .onFalse(new AlgaeArm.SetArmPositionCommand(AlgaeArmPosition.Up));
        algaeArmUp.onTrue(new AlgaeArm.SetArmPositionCommand(AlgaeArmPosition.Up));

        algaeHighElevator.whileTrue(new PickAlgaeSoonerCommand(Elevator.ElevatorSetpoint.AlgaeReefHigh)).onFalse(Subsystems.algaeIntake.holdAlgaeCommand());
        algaeLowElevator.whileTrue(new PickAlgaeSoonerCommand(Elevator.ElevatorSetpoint.AlgaeReefLow)).onFalse(Subsystems.algaeIntake.holdAlgaeCommand());

        climberPickup.onTrue(new Climber.ClimberMoveToPositionCommand(Climber.ClimberPosition.PICKUP));
        climberClimb.onTrue(new Climber.ClimberMoveToPositionCommand(Climber.ClimberPosition.CLIMB));

        manualStickControl.toggleOnTrue(Subsystems.algaeArm.openLoopCommand(manualAlgaeArmControl));
        manualStickControl.toggleOnTrue(Subsystems.elevator.openLoopCommand(manualElevatorControl));

        alignLeft.whileTrue(new AlignDriveInCommand(AlignDriveInCommand.AlignTarget.LEFT));
        alignRight.whileTrue(new AlignDriveInCommand(AlignDriveInCommand.AlignTarget.RIGHT));
        alignMiddle.whileTrue(new AlignDriveInCommand(AlignDriveInCommand.AlignTarget.CENTER));


        alignLeftStation.onTrue(Commands.runOnce(() -> swerveSupplier.setTargetHeading(
                GameInfo.isBlueAlliance() ? Degrees.of(-45) : Degrees.of(135))))
                .onFalse(Commands.runOnce(swerveSupplier::clearTargetHeading));
        alignRightStation.onTrue(Commands.runOnce(() -> swerveSupplier.setTargetHeading(
                GameInfo.isBlueAlliance() ? Degrees.of(45) : Degrees.of(-135))))
                .onFalse(Commands.runOnce(swerveSupplier::clearTargetHeading));

        enableClimbToggle.onTrue(Commands.runOnce(() -> climbEnabledState = !climbEnabledState));
        enableManualStickControlToggle.onTrue(Commands.runOnce(() -> manualStickControlState = !manualStickControlState));

        resetPoseButton.onTrue(new GenericPoseRequestCommand<>(ResetToAlliancePoseRequest.class));

        // overrideClimberUp.whileTrue(Subsystems.climber.openLoopUpDefault());
        // overrideClimberDown.whileTrue(Subsystems.climber.openLoopDownDefault());

        bindCommonButtons();
    }

    void bindCommonButtons() {
        SmartDashboard.putData("Reset Pose From Vision",
                VisionPoseUpdateFactory.resetFromMainPoseEstimator().ignoringDisable(true));

        SmartDashboard.putData("Reset Alliance Pose", new GenericPoseRequestCommand<>(ResetToAlliancePoseRequest.class));

        SmartDashboard.putData("Set LLs to Apriltag", new PipelineSwitcher(Pipeline.April));
        SmartDashboard.putData("Set LLs to Viewfinder", new PipelineSwitcher(Pipeline.View));
        SmartDashboard.putData("Clear LL ID Filter", Commands.runOnce(() -> Subsystems.visionSubsystem.resetIDFilter()));

        // reset the field-centric heading on left bumper press
        SmartDashboard.putData("Seed Field Centric", Commands.runOnce(() -> Subsystems.poseManager.pushRequest(new SeedFieldCentricRequest()))
                .ignoringDisable(true));

        SmartDashboard.putData("Zero Climber", Subsystems.climber.zeroClimberPosition().ignoringDisable(true));
        SmartDashboard.putData("Move Climber Up", new Climber.ClimberMoveToPositionCommand(Climber.ClimberPosition.UP));
        SmartDashboard.putData("Move Climber Down", new Climber.ClimberMoveToPositionCommand(Climber.ClimberPosition.DOWN));

        // AMD
        SmartDashboard.putData("Run All AMD", new RunAMDCommand());
        SmartDashboard.putData("Run DMS", new RunDMSCommand());
        SmartDashboard.putData("Stop AMD", Commands.runOnce(() -> Subsystems.ledSubsystem.getAMDSerialData().startAMDPhase(AMDSerialData.AMDPhase.Comm)).ignoringDisable(true));
        SmartDashboard.putData("Run CoralIntake AMD", new CoralIntakeAMDCommand());
        SmartDashboard.putData("Run Elevator AMD", new ElevatorAMDCommand());
        SmartDashboard.putData("Run AlgaeArm AMD", new AlgaeArmAMDCommand());
        SmartDashboard.putData("Run AlgaeIntake AMD", new AlgaeIntakeAMDCommand());

        amdButton.onTrue(new RunAMDCommand());

        // End AMD

        SmartDashboard.putData("Open Latch", Subsystems.funnelSubsystem.openLatchCommand().ignoringDisable(true));
        SmartDashboard.putData("Close Latch", Subsystems.funnelSubsystem.closeLatchCommand().ignoringDisable(true));

        SmartDashboard.putData("Debug Rotate", new RotateToAngleCommand(Degrees.of(120)));
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Controls/Climb Enabled", climbEnabledState);
        SmartDashboard.putBoolean("Controls/Manual Stick Control", manualStickControlState);
    }

    @Override
    public void teleopInit() {
        this.climbEnabledState = false;
        this.manualStickControlState = false;
        this.swerveSupplier.clearTargetHeading();
    }

    /**
     * This method is used to dampen the joystick input for the algae arm by remapping the joystick input to a smaller range
     * @return the transformed joystick input
     */
    Supplier<Double> algaeArmDampener() {
        Supplier<Double> dampener = () ->
                BSMath.map(joystick.getLeftY(), -1, 1, -0.1, 0.1);
        return deadband(dampener, 0.05);
    }
    
}