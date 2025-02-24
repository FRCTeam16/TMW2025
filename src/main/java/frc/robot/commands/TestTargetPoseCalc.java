package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.AprilTagUtil;

import java.util.Optional;

/**
 * Debug/Testing command to calculate the target pose for a given tag. This publishes the tag pose and the target pose
 * to NetworkTables for visualization or debugging.
 */
public class TestTargetPoseCalc extends Command {
    private final StructPublisher<Pose2d> tagPublisher;
    private final StructPublisher<Pose2d> targetPublisher;

    public TestTargetPoseCalc() {
       this(10);
    }

    public TestTargetPoseCalc(int aprilTagId) {
        tagPublisher = NetworkTableInstance.getDefault()
                .getStructTopic("PoseCalc/tag", Pose2d.struct).publish();
        targetPublisher = NetworkTableInstance.getDefault()
                .getStructTopic("PoseCalc/target", Pose2d.struct).publish();
        SmartDashboard.setDefaultNumber("PoseCalc/Tag", aprilTagId);
    }


    @Override
    public void execute() {
        int aprilTag = (int) SmartDashboard.getNumber("PoseCalc/Tag", 10);
        Optional<Pose2d> tagPose = Subsystems.aprilTagUtil.getTagPose2d(aprilTag);
        tagPose.ifPresent(tagPublisher::set);

        Optional<Pose2d> targetPose = Subsystems.aprilTagUtil.getScoringPoseForTag(aprilTag, true);
        targetPose.ifPresent(targetPublisher::set);
    }
}
