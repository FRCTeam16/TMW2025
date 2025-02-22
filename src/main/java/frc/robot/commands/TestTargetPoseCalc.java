package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.scoring.TargetPose;

import java.util.Optional;

public class TestTargetPoseCalc extends Command {
    private final StructPublisher<Pose2d> tagPublisher;
    private final StructPublisher<Pose2d> targetPublisher;

    public TestTargetPoseCalc() {
        tagPublisher = NetworkTableInstance.getDefault()
                .getStructTopic("PoseCalc/tag", Pose2d.struct).publish();
        targetPublisher = NetworkTableInstance.getDefault()
                .getStructTopic("PoseCalc/target", Pose2d.struct).publish();
        SmartDashboard.setDefaultNumber("PoseCalc/Tag", 10);
    }


    @Override
    public void execute() {
        int aprilTag = (int) SmartDashboard.getNumber("PoseCalc/Tag", 10);
        Optional<Pose3d> tagPose3d = Subsystems.visionSubsystem.getTagPose(aprilTag);
        if (tagPose3d.isPresent()) {
            Pose2d tagPose = tagPose3d.get().toPose2d();
            tagPublisher.set(tagPose);
        }

        Optional<Pose2d> targetPose = TargetPose.getScoringPoseForTag(aprilTag, true);
        targetPose.ifPresent(targetPublisher::set);
    }
}
