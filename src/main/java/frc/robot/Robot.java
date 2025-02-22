// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.RobotConfig;
import frc.robot.util.BSLogger;
import au.grapplerobotics.CanBridge;

import java.util.Queue;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  public static final RobotConfig robotConfig = RobotConfig.getInstance();
  private final RobotContainer m_robotContainer;
  public static Queue<Pose2d> poseUpdates = new java.util.LinkedList<>();

  private Alert resetPoseAlert = new Alert("Reset robot pose", Alert.AlertType.kInfo);

  public Robot() {
    m_robotContainer = new RobotContainer();
    CanBridge.runTCP();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Yaw", Subsystems.swerveSubsystem.getPigeon2().getYaw().getValueAsDouble());

    if (!poseUpdates.isEmpty()) {
      Pose2d pose = poseUpdates.poll();
      BSLogger.log("Robot", "Resetting pose to: " + pose);
//      resetPoseAlert.set(true);
      Subsystems.swerveSubsystem.resetPose(pose);
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
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
