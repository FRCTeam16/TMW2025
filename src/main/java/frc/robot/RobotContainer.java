// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.hci.control.ControlBinding;
import frc.robot.hci.control.ControlBindingFactory;
import frc.robot.hci.control.ControlBindingFactory.JoystickMode;
import frc.robot.hci.swerve.SwerveSupplier;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.vision.VisionAssist;

import java.util.Objects;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Constants.MaxSpeed;

public class RobotContainer {
    private RobotContainer instance;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    VisionAssist visionAssist = new VisionAssist(drive); // TODO: make real subsystemfile

    private final Telemetry logger = new Telemetry(MaxSpeed.in(MetersPerSecond));

    private final Joystick driveStick = Controls.left;
    private final Joystick steerStick = Controls.right;
    private final CommandXboxController joystick = Controls.joystick;

//    private final JoystickButton visionAssistButton = new JoystickButton(driveStick, 2);

    public final CommandSwerveDrivetrain drivetrain;
    private final SwerveSupplier swerveSupplier;

    private final JoystickMode joystickMode = JoystickMode.Bayou;
    private final ControlBinding controlBinding;

    public static RobotContainer getInstance() {
        return new RobotContainer();
    }

    private RobotContainer() {
        Subsystems.getInstance();   // Ensure subsystems are initialized
        controlBinding = ControlBindingFactory.createControlBinding(this.joystickMode, driveStick, steerStick, joystick);
        swerveSupplier = controlBinding.getSwerveSupplier();
        drivetrain = Subsystems.swerveSubsystem;

        configureDrivetrain();        // Configure control bindings before setting default command
        SmartDashboard.putString("Joystick Mode", this.joystickMode.toString());
    }

    private void configureDrivetrain() {
        // View that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {
                if(true/*!visionAssistButton.getAsBoolean()*/){ // use default swerve request unless visionAssist button pressed
                    return drive.withVelocityX(swerveSupplier.supplyX()) // Drive forward with negative Y (forward)
                        .withVelocityY(swerveSupplier.supplyY()) // Drive left with negative X (left)
                        .withRotationalRate(swerveSupplier.supplyRotationalRate()); // Drive counterclockwise with negative X (left)
                 }
                return visionAssist.deferDrive(swerveSupplier);
                }
            ).withName("Default Teleop")
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public SwerveSupplier getSwerveSupplier() {
        return swerveSupplier;
    }

    public Command getAutonomousCommand() {
        return Subsystems.autoManager.getSelectedAutoStrategy();
    }


    public void robotInit() {
        Subsystems.lifecycleSubsystems.stream().filter(Objects::nonNull).forEach(Lifecycle::robotInit);
        controlBinding.robotInit();
    }

    public void robotPeriodic() {
        controlBinding.periodic();
    }

    public void teleopInit() {
        Subsystems.lifecycleSubsystems.stream().filter(Objects::nonNull).forEach(Lifecycle::teleopInit);
        controlBinding.teleopInit();
    }

    public void autoInit() {
        Subsystems.lifecycleSubsystems.stream().filter(Objects::nonNull).forEach(Lifecycle::autoInit);
        controlBinding.autoInit();
    }

}
