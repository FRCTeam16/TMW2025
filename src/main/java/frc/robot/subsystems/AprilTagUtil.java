package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Subsystems;
import frc.robot.subsystems.scoring.TargetPose;

import java.util.Optional;

public class AprilTagUtil implements Sendable {
    private double scoringDistance = -0.5; // robot meters from tag
    private double offsetDistance = 0.165; // 0.17; // robot horizontal offset from tag


    private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public AprilTagUtil() {
    }

    public double getScoringDistance() {
        return scoringDistance;
    }

    public void setScoringDistance(double scoringDistance) {
        this.scoringDistance = scoringDistance;
    }

    public double getOffsetDistance() {
        return offsetDistance;
    }

    public void setOffsetDistance(double offsetDistance) {
        this.offsetDistance = offsetDistance;
    }

    @Override
    public void initSendable(SendableBuilder sendableBuilder) {
        sendableBuilder.setSmartDashboardType("AprilTagUtil");
        sendableBuilder.addDoubleProperty("scoringDistance", this::getScoringDistance, this::setScoringDistance);
        sendableBuilder.addDoubleProperty("offsetDistance", this::getOffsetDistance, this::setOffsetDistance);
    }

    public Optional<Pose2d> getScoringPoseForTag(int aprilTagID, boolean isLeft) {
        if (aprilTagID < 0) {
            return Optional.empty();
        }

        // Gets tag pose in blue alliance coordinates, we use the x/y values
        Pose2d tagPose2d = Subsystems.aprilTagUtil.getTagPose2d(aprilTagID).orElse(null);
        if (tagPose2d == null) {
            return Optional.empty();
        }

        // Calculate perpendicular angle for robot alignment
        Rotation2d scoreAngle = tagPose2d.getRotation().plus(Rotation2d.fromDegrees(180));

        // Calculate offset based on left/right scoring position
        double lateralOffset = isLeft ? -offsetDistance : offsetDistance; // meters

//        if (aprilTagID == 7) {
//            if (isLeft)
//                lateralOffset = -3.84;
//            else {
//                lateralOffset = 3.83;
//            }
//        }

        // Create transform from tag to scoring position
        Transform2d scoreTransform = new Transform2d(
                new Translation2d(-scoringDistance, lateralOffset),
                Rotation2d.fromDegrees(0)
        );

        // Apply transform to get final scoring position
        Pose2d translatedTarget = tagPose2d.transformBy(scoreTransform);
        Pose2d target = new Pose2d(translatedTarget.getTranslation(), scoreAngle);
        return Optional.of(target);
    }

    public Optional<Pose3d> getTagPose(int aprilTagID) {
        return this.fieldLayout.getTagPose(aprilTagID);
    }

    public Optional<Pose2d> getTagPose2d(int aprilTagID) {
        return this.fieldLayout.getTagPose(aprilTagID).map(Pose3d::toPose2d);
    }
}
