package frc.robot.auto.strategies;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.commands.pose.GenericPoseRequestCommand;
import frc.robot.commands.vision.AlignDriveInCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.pose.UpdateTranslationFromVision;
import frc.robot.util.BSLogger;
import frc.robot.util.GameInfo;

public class ArkansasStrategy extends AutoPathStrategy {

    public ArkansasStrategy(boolean isLeft, boolean isRed) {

        final Pose2d firstScorePose;
        final Pose2d coralStationPose;
        final Pose2d secondScorePose;

        // Baseline coordinates
        final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        final double FW =  fieldLayout.getFieldWidth();
        final double FL = fieldLayout.getFieldLength();

        final Pose2d rr_firstScorePose = new Pose2d(12.275, 5.4, Rotation2d.fromDegrees(-60));
        final Pose2d rr_coralStationPose = new Pose2d(16.45, 7.75, Rotation2d.fromDegrees(-125));
        final Pose2d rr_secondScorePose = new Pose2d(14.4, 5.52, Rotation2d.fromDegrees(-120));

        final Pose2d rl_firstScorePose = new Pose2d(12.275, FW - 5.4, Rotation2d.fromDegrees(60));
        final Pose2d rl_coralStationPose = new Pose2d(16.45, FW - 7.75, Rotation2d.fromDegrees(125));
        final Pose2d rl_secondScorePose = new Pose2d(14.4, FW - 5.52, Rotation2d.fromDegrees(120));

        final Pose2d br_firstScorePose = new Pose2d(FL - 12.275, FW - 5.4, Rotation2d.fromDegrees(120));
        final Pose2d br_coralStationPose = new Pose2d(FL - 16.45, FW - 7.75, Rotation2d.fromDegrees(45));
        final Pose2d br_secondScorePose = new Pose2d(FL - 14.4, FW - 5.52, Rotation2d.fromDegrees(60));

        final Pose2d bl_firstScorePose = new Pose2d(FL - 12.275, 5.4, Rotation2d.fromDegrees(-120));
        final Pose2d bl_coralStationPose = new Pose2d(FL - 16.45, 7.75, Rotation2d.fromDegrees(-45));
        final Pose2d bl_secondScorePose = new Pose2d(FL - 14.4, 5.52, Rotation2d.fromDegrees(-60));



        if (isRed && !isLeft) {
            // Red Right
            firstScorePose = rr_firstScorePose;
            coralStationPose = rr_coralStationPose;
            secondScorePose = rr_secondScorePose;
        } else if (isRed && isLeft) {
            // Red Left
            firstScorePose = rl_firstScorePose;
            coralStationPose = rl_coralStationPose;
            secondScorePose = rl_secondScorePose;
        } else if (!isRed && !isLeft) {
            // Blue Right
            firstScorePose = br_firstScorePose;
            coralStationPose = br_coralStationPose;
            secondScorePose = br_secondScorePose;
        } else /*if (!isRed && isLeft) */ {
            // Blue Left
            firstScorePose = bl_firstScorePose;
            coralStationPose = bl_coralStationPose;
            secondScorePose = bl_secondScorePose;
        }

        final Pose2d simulationPose;
        Pose2d sim_red_right = new Pose2d(10, 5.6, Rotation2d.fromDegrees(135));
        Pose2d sim_red_left = new Pose2d(10, 3, Rotation2d.fromDegrees(-135));
        if (GameInfo.isRedAlliance()) {
            simulationPose = isLeft ? sim_red_left : sim_red_right;
        } else {
            simulationPose = isLeft ?
                    FlippingUtil.flipFieldPose(sim_red_left) :
                    FlippingUtil.flipFieldPose(sim_red_right);
        }


        BSLogger.log("ArkansasStrat", "Arkansas Strategy: " + (isRed ? "Red" : "Blue") + " " + (isLeft ? "Left" : "Right"));
        BSLogger.log("ArkansasStrat", "First Score Pose: " + firstScorePose);
        BSLogger.log("ArkansasStrat", "Coral Station Pose: " + coralStationPose);
        BSLogger.log("ArkansasStrat", "Second Score Pose: " + secondScorePose);

        Command initialPoseCommand = RobotBase.isReal() ?
                new GenericPoseRequestCommand<>(UpdateTranslationFromVision.class) :
                Commands.runOnce(() -> Subsystems.swerveSubsystem.resetPose(simulationPose));



        addCommands(
                //
                // Initial setup
                //
                initialPoseCommand,
//                new GenericPoseRequestCommand<>(UpdateTranslationFromVision.class),
                new Climber.ClimberMoveToPositionNoWait(Climber.ClimberPosition.DOWN),

                //
                // Go to first score pose
                //
                //  Subsystems.autoManager.pathfindThenFollowPathCommand(key+"1"),
                Commands.runOnce(() -> BSLogger.log("ArkansasStrat", "Path to pose: " + firstScorePose)),
                AutoBuilder.pathfindToPose(firstScorePose, Constants.pathConstraints, 0.5),
                new Climber.ClimberMoveToPositionNoWait(Climber.ClimberPosition.DOWN),

                doScoreSequence(false),

                //
                // Go to coral station

                Commands.runOnce(Subsystems.coralIntake::startIntakeAuto),
                Commands.runOnce(() -> BSLogger.log("ArkansasStrat", "Path to pose: " + coralStationPose)),
                AutoBuilder.pathfindToPose(coralStationPose, Constants.pathConstraints),

                //
                // Go to second score pose
                //
                Commands.runOnce(() -> BSLogger.log("ArkansasStrat", "Path to pose: " + secondScorePose)),
                AutoBuilder.pathfindToPose(secondScorePose, Constants.pathConstraints, 0.5),
                Commands.runOnce(Subsystems.coralIntake::stopIntakeAuto),
                doScoreSequence(false),

                Commands.print("Finished Arkansas Left Strategy")
        );
    }

    // FIXME: Don't worry about doing this if we don't have a coral
    public Command doScoreSequence(boolean isLeft) {
        return Commands.sequence(
                        new Climber.ClimberMoveToPositionNoWait(Climber.ClimberPosition.DOWN),
                        new AlignDriveInCommand(isLeft).withTimeout(0.75),
                        new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L4).withTimeout(5),
                        Subsystems.coralIntake.shootCoralCommand().withTimeout(0.5),
                        Commands.parallel(
                                Subsystems.coralIntake.stopCommand(),
                                new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.Zero).withTimeout(1)
                        )
                )
                .withName("Scoring Sequence");
//                .unless(!Subsystems.coralIntake.coralDetectedAtBottomSensor())
    }
}
