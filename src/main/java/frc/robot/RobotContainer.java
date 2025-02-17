// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.MaxAngularRate;
import static frc.robot.Constants.MaxSpeed;

import java.util.Objects;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.JoystickMode;
import frc.robot.commands.AlignmentTest;
import frc.robot.commands.ResetPoseCommand;
import frc.robot.commands.ZeroYawCommand;
import frc.robot.commands.auto.PathfindToPoseCommand;
import frc.robot.commands.vision.PipelineSwitcher;
import frc.robot.commands.vision.UpdateRobotPoseFromVision;
import frc.robot.hci.JoystickSwerveSupplier;
import frc.robot.hci.SwerveSupplier;
import frc.robot.hci.XBoxSwerveSupplier;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.Intake.AlgaeArm;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.Prototype.ComponentPreconfig;
import frc.robot.subsystems.vision.Pipeline;
import frc.robot.subsystems.vision.VisionAssist;

public class RobotContainer {

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed.in(MetersPerSecond) * 0.125)
            .withRotationalDeadband(MaxAngularRate.in(RadiansPerSecond) * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    VisionAssist visionAssist = new VisionAssist(drive); // TODO: make real subsystemfile

    private final Telemetry logger = new Telemetry(MaxSpeed.in(MetersPerSecond));

    private final Joystick driveStick = Controls.left;
    private final Joystick steerStick = Controls.right;
    private final CommandXboxController joystick = Controls.joystick;

    private final JoystickButton prototypeButton = new JoystickButton(driveStick, 1); // for prototype subsystem
    private final JoystickButton visionAssistButton = new JoystickButton(driveStick, 2);

    public final CommandSwerveDrivetrain drivetrain;
    private final SwerveSupplier swerveSupplier;

    private Constants.JoystickMode joystickMode = JoystickMode.none;

    public RobotContainer() {
        Subsystems.getInstance(); // Ensure subsystems are initialized
        drivetrain = Subsystems.swerveSubsystem;
        swerveSupplier = (!RobotBase.isSimulation()) ?
                new JoystickSwerveSupplier(driveStick, steerStick, joystick) :
                new XBoxSwerveSupplier(joystick);

        if (RobotBase.isSimulation()) {
            drivetrain.resetPose(new Pose2d(3, 3, Rotation2d.fromDegrees(0)));
        }

        configureBindings();
    }

    private void configureBindings() {
        // View that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {
                if(!visionAssistButton.getAsBoolean()){ // use default swerve request unless visionAssist button pressed
                    return drive.withVelocityX(swerveSupplier.supplyX()) // Drive forward with negative Y (forward)
                        .withVelocityY(swerveSupplier.supplyY()) // Drive left with negative X (left)
                        .withRotationalRate(swerveSupplier.supplyRotationalRate()); // Drive counterclockwise with negative X (left)
                 }
                return visionAssist.deferDrive(swerveSupplier);
                }
            ).withName("Default Teleop")
        );

        switch (this.joystickMode) {
            case JoshPrototype -> {
                // Josh Prototype Controls
                joystick.b().onTrue(Subsystems.joshPrototype.stop());
                joystick.y().onTrue(Subsystems.joshPrototype.eject()).onFalse(Subsystems.joshPrototype.stop());
                joystick.a().onTrue(Subsystems.joshPrototype.ingest()).onFalse(Subsystems.joshPrototype.stop());
            }
            case AustinGearboxPrototype -> {
                Subsystems.austinGearPrototype.InjectControls(ComponentPreconfig.ABXYpreconf);
            }
            case AlignmentTest -> {
                joystick.x().whileTrue(new AlignmentTest(AlignmentTest.TargetSide.LEFT));
                joystick.b().whileTrue(new AlignmentTest(AlignmentTest.TargetSide.RIGHT));
            }
            case ElevatorProto -> {
                joystick.a().onTrue(Subsystems.elevator.openLoopDownCommand()).onFalse(Subsystems.elevator.openLoopStopCommand());
                joystick.y().onTrue(Subsystems.elevator.openLoopUpCommand()).onFalse(Subsystems.elevator.openLoopStopCommand());
            }
            case climberProto -> { // dual integrated motors
                joystick.rightBumper().and(joystick.a()).onTrue(Subsystems.Climberproto1.runForward()).onFalse(Subsystems.Climberproto1.stop());
                joystick.rightBumper().and(joystick.y()).onTrue(Subsystems.Climberproto1.runBackward()).onFalse(Subsystems.Climberproto1.stop());
                joystick.leftBumper().and(joystick.a()).onTrue(Subsystems.Climberproto2.runForward()).onFalse(Subsystems.Climberproto2.stop());
                joystick.leftBumper().and(joystick.y()).onTrue(Subsystems.Climberproto2.runBackward()).onFalse(Subsystems.Climberproto2.stop());
                joystick.b().onTrue(Subsystems.Climberproto1.stop());
                joystick.b().onTrue(Subsystems.Climberproto2.stop());
            }
            case AlgaeProto -> {
                joystick.y().onTrue(Subsystems.algaeIntake.intakeCommand()).onFalse(Subsystems.algaeIntake.holdAlgaeCommand());
                joystick.x().onTrue(Subsystems.algaeIntake.ejectCommand()).onFalse(Subsystems.algaeIntake.stopCommand());
//                joystick.a().onTrue(Subsystems.algaeArm.setArmPositionCommand(AlgaeArm.AlgaeArmPosition.ReefLow));
//                joystick.y().onTrue(Subsystems.algaeArm.setArmPositionCommand(AlgaeArm.AlgaeArmPosition.ReefHigh));
            }
            case none -> {

            }
        }

        // Debug Testing
        Pose2d targetPose = new Pose2d(3.39, 4.05, Rotation2d.fromDegrees(180));
        PathConstraints pathConstraints = new PathConstraints(1.0, 1.0, 1.0, 1.0);
        Command cmd = Commands.runOnce(() -> AutoBuilder.pathfindToPose(targetPose, pathConstraints).schedule(), Subsystems.swerveSubsystem);
        joystick.rightBumper().onTrue(cmd).onFalse(Commands.run(cmd::cancel));
        // Debug Testing// Debug Testing

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        bindSmartDashboardButtons();

        drivetrain.registerTelemetry(logger::telemeterize);

        bindSysId();

        joystick.povLeft().and(joystick.a()).onTrue(Subsystems.coralIntake.intakeCoralCommand());
        joystick.povLeft().and(joystick.b()).onTrue(Subsystems.coralIntake.stopCommand());
//        joystick.povLeft().and(joystick.y()).whileTrue(Subsystems.coralIntake.ejectCommand());
//        joystick.povLeft().and(joystick.x()).whileTrue(Subsystems.coralIntake.shootCoralCommand());

    }

    public void bindSysId() {
        // Run SysId routines when holding back/start and X/Y. // these are fine for now
        // View that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.getSysIdHelper().sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.getSysIdHelper().sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.getSysIdHelper().sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.getSysIdHelper().sysIdQuasistatic(Direction.kReverse));
    }

    public void bindSmartDashboardButtons() {
        SmartDashboard.putData("Zero Yaw", new ZeroYawCommand());
        SmartDashboard.putData("Set LLs to Apriltag", new PipelineSwitcher(Pipeline.April));
        SmartDashboard.putData("Set LLs to Viewfinder", new PipelineSwitcher(Pipeline.View));

        SmartDashboard.putData("Test Reset Pose", new ResetPoseCommand());
        Subsystems.visionSubsystem.getLimelights().forEach(limelight ->
                SmartDashboard.putData("Test Reset Pose from " + limelight.getName(),
                UpdateRobotPoseFromVision.resetFromLimelightPoseEstimator(limelight.getName())));

        // Debug Testing
        SmartDashboard.putData("Pathfind", new PathfindToPoseCommand());
        SmartDashboard.putData("ResetPoseTest", Commands.runOnce(
            () ->{
                // Locking robot code when called from a command
                System.out.println("PRE RESET");
//                drivetrain.resetPose(new Pose2d(7, 6, Rotation2d.fromDegrees(0)));
                Robot.poseUpdates.add(new Pose2d(7, 6, Rotation2d.fromDegrees(0)));
                System.out.println("POST RESET");
            },
                Subsystems.swerveSubsystem)
                .withName("RESET POSE TEST")
                .withTimeout(5.0)
                .ignoringDisable(true)
        );
        // Debug Testing
    }

    public Command getAutonomousCommand() {
        return Subsystems.autoManager.getSelectedAutoStrategy();
    }

    public void teleopInit() {
        Subsystems.lifecycleSubsystems.stream().filter(Objects::nonNull).forEach(Lifecycle::teleopInit);
    }

    public void autoInit() {
        Subsystems.lifecycleSubsystems.stream().filter(Objects::nonNull).forEach(Lifecycle::autoInit);
    }
}
