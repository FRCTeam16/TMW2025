// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.hci.SwerveSupplier.MaxAngularRate;
import static frc.robot.hci.SwerveSupplier.MaxSpeed;

import java.util.Objects;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.vision.VisionAssist;

import frc.robot.hci.JoystickSwerveSupplier;
import frc.robot.hci.SwerveSupplier;
import frc.robot.hci.XBoxSwerveSupplier;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Lifecycle;

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

        // Josh Prototype Controls
        joystick.b().onTrue(Subsystems.joshPrototype.stop());
        joystick.y().onTrue(Subsystems.joshPrototype.eject()).onFalse(Subsystems.joshPrototype.stop());
        joystick.a().onTrue(Subsystems.joshPrototype.ingest()).onFalse(Subsystems.joshPrototype.stop());

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

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
