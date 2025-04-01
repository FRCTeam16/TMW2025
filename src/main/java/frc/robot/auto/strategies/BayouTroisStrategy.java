package frc.robot.auto.strategies;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.DriveRobotCentricCommand;
import frc.robot.commands.RotateToAngleCommand;
import frc.robot.commands.path.ProfiledDriveCommand;
import frc.robot.commands.pose.GenericPoseRequestCommand;
import frc.robot.commands.vision.AlignDriveInCommand;
import frc.robot.commands.vision.AlignDriveInCommand.AlignTarget;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorSetpoint;
import frc.robot.subsystems.pose.UpdateTranslationFromVision;
import frc.robot.subsystems.vision.LimelightHelpers;

import static edu.wpi.first.units.Units.*;

public class BayouTroisStrategy extends AutoPathStrategy {

    // TODO: this is start of path planner Pose2d simulationPose = new Pose2d(12.37,
    // 5.03, Rotation2d.fromDegrees(-60));

    public enum StartingPosition {
        RED_RIGHT,
        RED_LEFT,
        BLUE_RIGHT,
        BLUE_LEFT
    }

    // Baseline coordinates
    final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    final double FW = fieldLayout.getFieldWidth();
    final double FL = fieldLayout.getFieldLength();

    // Target Pose
    final Pose2d rr_targetPose = new Pose2d(12.275, 5.4, Rotation2d.fromDegrees(-60));
    final Pose2d rl_targetPose = new Pose2d(12.275, FW - 5.4, Rotation2d.fromDegrees(60));
    private Pose2d br_targetPose = FlippingUtil.flipFieldPose(rr_targetPose);
    private Pose2d bl_targetPose = new Pose2d(FL - 12.275, 5.4, Rotation2d.fromDegrees(-120));

    // Simulation Pose
    Pose2d rr_simulationPose = new Pose2d(10, 5.6, Rotation2d.fromDegrees(-60));
    Pose2d rl_simulationPose = new Pose2d(10, FW - 5.6, Rotation2d.fromDegrees(60));
    Pose2d br_simulationPose = FlippingUtil.flipFieldPose(rr_simulationPose);
    Pose2d bl_simulationPose = new Pose2d(FL - 10, 5.6, Rotation2d.fromDegrees(-120));

    public BayouTroisStrategy(StartingPosition startingPosition) {
        final Pose2d targetPose = switch (startingPosition) {
            case RED_RIGHT -> rr_targetPose;
            case RED_LEFT -> rl_targetPose;
            case BLUE_RIGHT -> br_targetPose;
            case BLUE_LEFT -> bl_targetPose;
        };

        final Pose2d simulationPose = switch (startingPosition) {
            case RED_RIGHT -> rr_simulationPose;
            case RED_LEFT -> rl_simulationPose;
            case BLUE_RIGHT -> br_simulationPose;
            case BLUE_LEFT -> bl_simulationPose;
        };

        final String firstDrive = switch (startingPosition) {
            case RED_RIGHT -> "B3FirstDriveRight";
            case RED_LEFT -> "B3FirstDriveLeft";
            case BLUE_RIGHT -> "B3FirstDriveRight";
            case BLUE_LEFT -> "B3FirstDriveLeft";
        };

        final int[] ALL_IDS = new int[] {};
        final int[] firstDriveFilters = switch (startingPosition) {
            case RED_RIGHT -> new int[] { 9 };
            case RED_LEFT -> new int[] { 11 };
            case BLUE_RIGHT -> new int[] { 22 };
            case BLUE_LEFT -> ALL_IDS;
        };

        final String secondDrive = switch (startingPosition) {
            case RED_RIGHT -> "B3SecondDriveRight";
            case RED_LEFT -> "B3SecondDriveLeft";
            case BLUE_RIGHT -> "B3SecondDriveRight";
            case BLUE_LEFT -> "B3SecondDriveLeft";
        };

        final String thirdDrive = switch (startingPosition) {
            case RED_RIGHT -> "B3ThirdDriveRight";
            case RED_LEFT -> "B3ThirdDriveLeft";
            case BLUE_RIGHT -> "B3ThirdDriveRight";
            case BLUE_LEFT -> "B3ThirdDriveLeft";
        };

        Command initialPoseCommand = RobotBase.isReal()
                ? new GenericPoseRequestCommand<>(UpdateTranslationFromVision.class)
                : Commands.runOnce(() -> Subsystems.swerveSubsystem.resetPose(simulationPose));

        addCommands(
//                Commands.runOnce(() -> LimelightHelpers.SetFiducialIDFiltersOverride("limelight", firstDriveFilters)),
//                new WaitCommand(0.1),
//                initialPoseCommand,
//                new ProfiledDriveCommand(targetPose)
//                        .withTolerance(Meters.of(0.5))
//                        .withFinalState(new State(0, 1.0)),
//                runAutoPath(firstDrive),

                new DriveRobotCentricCommand(Seconds.of(1.1)),
                new RotateToAngleCommand(targetPose.getRotation().getMeasure()).withTimeout(0.5),
                new AlignDriveInCommand(AlignTarget.LEFT).withTimeout(1.5),
                doScoreSequence(),
                Commands.runOnce(() -> Subsystems.visionSubsystem.resetIDFilter()),

                runAutoPath(secondDrive),
                new AlignDriveInCommand(AlignTarget.RIGHT).withTimeout(1.5),
                doScoreSequence(),

                runAutoPath(thirdDrive),
                new AlignDriveInCommand(AlignTarget.LEFT).withTimeout(1.5),
                doScoreSequence(),

                // Let commands finish if we have time leftover, FMS will kill this for us
                new PrintCommand("Done with Bayou Trois"),
                new WaitCommand(3)
        );
    }

    public Command doScoreSequence() {
        return Commands.sequence(
                new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L4).withTimeout(1.5),
                Subsystems.coralIntake.shootCoralCommand().withTimeout(0.5),
                Commands.parallel(
                        Subsystems.coralIntake.stopCommand(),
                        new Elevator.ElevatorMoveToPositionCommand(ElevatorSetpoint.Zero).withNoWait(),
                        new WaitCommand(0.6)
                )
        ).withName("Scoring Sequence");
    }

}
