// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.RobotConfig;
import frc.robot.subsystems.vision.Pipeline;
import frc.robot.util.BSLogger;
import frc.robot.util.GameInfo;

import java.util.Queue;

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
  }

  @Override
  public void robotInit() {

    // Set up starting config
    if (GameInfo.isRedAlliance()) {
      BSLogger.log("Robot", "robotInit:: setting pose for red");
//      Subsystems.swerveSubsystem.getPigeon2().setYaw(0);
//      Subsystems.swerveSubsystem.resetPose(new Pose2d(14, 7.3, Rotation2d.fromDegrees(0)));
    } else if (GameInfo.isBlueAlliance()){
      BSLogger.log("Robot", "robotInit:: setting pose for red");
//      Subsystems.swerveSubsystem.getPigeon2().setYaw(0);
//      Subsystems.swerveSubsystem.resetPose(new Pose2d(8, 3, Rotation2d.fromDegrees(0)));
    } else {
      BSLogger.log("Robot", "robotInit:: setting pose for unknown alliance");
      Subsystems.swerveSubsystem.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Yaw", Subsystems.swerveSubsystem.getPigeon2().getYaw().getValueAsDouble() % 360.0);
    SmartDashboard.putNumber("Rot", Subsystems.swerveSubsystem.getState().Pose.getRotation().getDegrees());
    Subsystems.poseManager.update();
  }

  @Override
  public void disabledInit() {
    // We may want to switch to the view pipeline when disabled for thermal reasons
//    Subsystems.visionSubsystem.selectPipeline(Pipeline.View);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
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
    Subsystems.visionSubsystem.selectPipeline(Pipeline.April);
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
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
