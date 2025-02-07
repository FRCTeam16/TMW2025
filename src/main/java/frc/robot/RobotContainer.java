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

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ResetPoseCommand;
import frc.robot.commands.ZeroYawCommand;
import frc.robot.commands.vision.UpdateRobotPoseFromVision;
import frc.robot.hci.JoystickSwerveSupplier;
import frc.robot.hci.SwerveSupplier;
import frc.robot.hci.XBoxSwerveSupplier;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.vision.VisionAssist;
import frc.robot.commands.AlignmentTest;

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

    private final Joystick driveStick = new Joystick(0);
    private final Joystick steerStick = new Joystick(1);
    private final CommandXboxController joystick = new CommandXboxController(2);

    private final JoystickButton prototypeButton = new JoystickButton(driveStick, 1); // for prototype subsystem
    private final JoystickButton visionAssistButton = new JoystickButton(driveStick, 2);
    

    public final CommandSwerveDrivetrain drivetrain;

    private final SwerveSupplier swerveSupplier;

    private Constants.JoystickMode joystickMode = Constants.JoystickMode.AlgaeProto;

    public RobotContainer() {
        Subsystems.getInstance(); // Ensure subsystems are initialized
        drivetrain = Subsystems.swerveSubsystem;
        swerveSupplier = (!RobotBase.isSimulation()) ?
                new JoystickSwerveSupplier(driveStick, steerStick, joystick) :
                new XBoxSwerveSupplier(joystick);
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically

                drivetrain.applyRequest(() -> {
                if(!visionAssistButton.getAsBoolean()){ // use default swerve request unless visionAssist button pressed
                    return drive.withVelocityX(swerveSupplier.supplyX()) // Drive forward with negative Y (forward)
                        .withVelocityY(swerveSupplier.supplyY()) // Drive left with negative X (left)
                        .withRotationalRate(swerveSupplier.supplyRotationalRate()); // Drive counterclockwise with negative X (left)
                 }
                return visionAssist.deferDrive(swerveSupplier);
                }
            )
        );

        switch (this.joystickMode) {
            case JoshPrototype -> {
                // Josh Prototype Controls
                joystick.b().onTrue(Subsystems.joshPrototype.stop());
                joystick.y().onTrue(Subsystems.joshPrototype.eject()).onFalse(Subsystems.joshPrototype.stop());
                joystick.a().onTrue(Subsystems.joshPrototype.ingest()).onFalse(Subsystems.joshPrototype.stop());
            }
            case AustinGearboxPrototype -> {
                joystick.b().onTrue(Subsystems.austinGearPrototype.stop());
                joystick.a().onTrue(Subsystems.austinGearPrototype.runForward()).onFalse(Subsystems.austinGearPrototype.stop());
                joystick.y().onTrue(Subsystems.austinGearPrototype.runBackward()).onFalse(Subsystems.austinGearPrototype.stop());
                joystick.x().onTrue(Subsystems.austinGearPrototype.updateIds()); //IDs are ran through elastic
            }
            case AlignmentTest -> {
                joystick.x().whileTrue(new AlignmentTest(AlignmentTest.TargetSide.LEFT));
                joystick.b().whileTrue(new AlignmentTest(AlignmentTest.TargetSide.RIGHT));
            }
            case ElevatorProto -> {
                joystick.a().onTrue(Subsystems.elevator.openLoopDown()).onFalse(Subsystems.elevator.openLoopStop());
                joystick.y().onTrue(Subsystems.elevator.openLoopUp()).onFalse(Subsystems.elevator.openLoopStop());
                joystick.x().onTrue(Subsystems.elevator.updateMotorIds());
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
                joystick.x().onTrue(Subsystems.algaeIntake.runForward()).onFalse(Subsystems.algaeIntake.hold());
                joystick.b().onTrue(Subsystems.algaeIntake.runBackward()).onFalse(Subsystems.algaeIntake.stop());
            }
        }


        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        bindSmartDashboardButtons();

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void bindSysId() {
        // Run SysId routines when holding back/start and X/Y. // these are fine for now
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.getSysIdHelper().sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.getSysIdHelper().sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.getSysIdHelper().sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.getSysIdHelper().sysIdQuasistatic(Direction.kReverse));
    }

    public void bindSmartDashboardButtons() {
        SmartDashboard.putData("Zero Yaw", new ZeroYawCommand());

        SmartDashboard.putData("Test Reset Pose", new ResetPoseCommand());
        Subsystems.visionSubsystem.getLimelights().forEach(limelight ->
                SmartDashboard.putData("Test Reset Pose from " + limelight.getName(),
                UpdateRobotPoseFromVision.resetFromLimelightPoseEstimator(limelight.getName())));
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
