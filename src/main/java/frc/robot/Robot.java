// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.RobotConfig;
import frc.robot.subsystems.vision.LimelightHelpers;
import frc.robot.subsystems.vision.Pipeline;
import frc.robot.util.BSLogger;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  public static final RobotConfig robotConfig = RobotConfig.getInstance();
  private final RobotContainer m_robotContainer;


  public Robot() {
    DataLogManager.start();
    m_robotContainer = RobotContainer.getInstance();
    CanBridge.runTCP();

    // Setup serial communications
    addPeriodic(Subsystems.ledSubsystem::Report, 0.1);

    if (RobotBase.isSimulation()) {
      BSLogger.log("Robot", "silencing joystick during simulation");
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  @Override
  public void robotInit() {

    // Allow other interested parties to respond
    m_robotContainer.robotInit();

    // Finally start warmup
    // DO THIS AFTER CONFIGURATION OF YOUR DESIRED PATHFINDER
    PathfindingCommand.warmupCommand().schedule();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Yaw", Subsystems.swerveSubsystem.getPigeonYaw());
    SmartDashboard.putNumber("Rot", Subsystems.swerveSubsystem.getState().Pose.getRotation().getDegrees());
    m_robotContainer.robotPeriodic();
    Subsystems.poseManager.update();
  }

  @Override
  public void disabledInit() {
    // We may want to switch to the view pipeline when disabled for thermal reasons
   Subsystems.visionSubsystem.selectPipeline(Pipeline.April);
   Subsystems.visionSubsystem.getLimelights().forEach(limelight ->
           LimelightHelpers.SetIMUMode(limelight.getName(), 1));
  }

  @Override
  public void disabledPeriodic() {
    Subsystems.visionOdometryUpdater.updateOdometry();
    Subsystems.poseManager.updateInitialStartingPoseAndGyro();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    Subsystems.visionSubsystem.getLimelights().forEach(limelight ->
            LimelightHelpers.SetIMUMode(limelight.getName(), 2));

    Subsystems.visionSubsystem.selectPipeline(Pipeline.April);
    BSLogger.log("Robot", "autoInit:: Started at:" + Timer.getFPGATimestamp());
    autonomousCommand = m_robotContainer.getAutonomousCommand();
    BSLogger.log("Robot", "autoInit:: got robotCommand: " + Timer.getFPGATimestamp());
    m_robotContainer.autoInit();
    BSLogger.log("Robot", "autoInit:: robot container autoInit finished: " + Timer.getFPGATimestamp());

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
      BSLogger.log("Robot", "autoInit:: scheduled command at: " + Timer.getFPGATimestamp());
    }
    BSLogger.log("Robot", "autoInit:: finished at: " + Timer.getFPGATimestamp());
  }

  @Override
  public void autonomousPeriodic() {
    Subsystems.visionOdometryUpdater.updateOdometry();
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    Subsystems.visionSubsystem.getLimelights().forEach(limelight ->
            LimelightHelpers.SetIMUMode(limelight.getName(), 2));

    Subsystems.visionSubsystem.selectPipeline(Pipeline.April);
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    m_robotContainer.teleopInit();
  }

  @Override
  public void teleopPeriodic() {
    Subsystems.visionOdometryUpdater.updateOdometry();
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
