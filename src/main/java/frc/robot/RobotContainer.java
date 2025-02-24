// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Constants.MaxSpeed;

import java.util.Objects;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.hci.control.ControlBindingFactory.JoystickMode;
import frc.robot.hci.control.ControlBinding;
import frc.robot.hci.control.ControlBindingFactory;
import frc.robot.hci.swerve.JoystickSwerveSupplier;
import frc.robot.hci.swerve.SwerveSupplier;
import frc.robot.hci.swerve.XBoxSwerveSupplier;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.vision.VisionAssist;
import frc.robot.util.GameInfo;

public class RobotContainer {
    private RobotContainer instance;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
//            .withDeadband(MaxSpeed.in(MetersPerSecond) * 0.125)
//            .withRotationalDeadband(MaxAngularRate.in(RadiansPerSecond) * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    VisionAssist visionAssist = new VisionAssist(drive); // TODO: make real subsystemfile

    private final Telemetry logger = new Telemetry(MaxSpeed.in(MetersPerSecond));

    private final Joystick driveStick = Controls.left;
    private final Joystick steerStick = Controls.right;
    private final CommandXboxController joystick = Controls.joystick;

    private final JoystickButton visionAssistButton = new JoystickButton(driveStick, 2);

    public final CommandSwerveDrivetrain drivetrain;
    private final SwerveSupplier swerveSupplier;

    private JoystickMode joystickMode = JoystickMode.Scrimmage;
    private ControlBinding controlBinding;

    public static RobotContainer getInstance() {
        return new RobotContainer();
    }

    private RobotContainer() {
        Subsystems.getInstance(); // Ensure subsystems are initialized
        drivetrain = Subsystems.swerveSubsystem;
        swerveSupplier = (!RobotBase.isSimulation()) ?
                new JoystickSwerveSupplier(driveStick, steerStick, joystick) :
                new XBoxSwerveSupplier(joystick);


        configureBindings();

        // Set up starting config
        if (GameInfo.isRedAlliance()) {
            drivetrain.resetPose(new Pose2d(12, 7.3, Rotation2d.fromDegrees(0)));
//            drivetrain.setOperatorPerspectiveForward(Rotation2d.fromDegrees(180));
        } else {
            drivetrain.resetPose(new Pose2d(3, 3, Rotation2d.fromDegrees(180)));
//            drivetrain.setOperatorPerspectiveForward(Rotation2d.fromDegrees(0));
        }

        SmartDashboard.putString("Joystick Mode", this.joystickMode.toString());

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

        controlBinding = ControlBindingFactory.createControlBinding(this.joystickMode, driveStick, steerStick, joystick);
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public SwerveSupplier getSwerveSupplier() {
        return swerveSupplier;
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
