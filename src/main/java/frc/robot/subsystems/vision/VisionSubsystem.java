package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Lifecycle;
import frc.robot.util.BSLogger;
import frc.robot.util.GameInfo;

import java.util.*;

public class VisionSubsystem extends SubsystemBase implements Lifecycle {
    private final Map<String, Limelight> limelightLookup = new HashMap<>();
    private final Limelight defaultLimelight;
    private final AprilTagFieldLayout fieldLayout;

    public VisionSubsystem(Iterable<Limelight> limelights) {
        Limelight tmpLimelight = null;
        for (Limelight limelight : limelights) {
            limelightLookup.put(limelight.getName(), limelight);
            if (tmpLimelight == null) {
                tmpLimelight = limelight;
            }
        }
        if (tmpLimelight != null) {
            this.defaultLimelight = tmpLimelight;
        } else {
            this.defaultLimelight = null;
            BSLogger.log("VisionSubsystem", "No Limelights were added to the VisionSubsystem");
        }
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);   // US regionals use welded

    }


    public Optional<Limelight> getDefaultLimelight() {
        return Optional.ofNullable(defaultLimelight);
    }

    public List<Limelight> getLimelights() {
        return new ArrayList<>(limelightLookup.values());
    }

    public Limelight getLimelightByName(String name) {
        Limelight limelight = limelightLookup.get(name);
        if (limelight != null) {
            return limelight;
        } else {
            String message = "ERROR: No limelight named %s was added to the VisionSubsystem".formatted(name);
            DataLogManager.log(message);
            throw new IllegalArgumentException(message);
        }
    }

    /**
     * Determines the best target info from all limelights
     *
     * @return target info
     */
    public Optional<VisionTypes.TargetInfo> getTargetInfo() {
        return getLimelights().stream()
                .map(Limelight::getTargetInfo)
                .max(Comparator.comparing(VisionTypes.TargetInfo::targetArea));

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("VisionSubsystem");
        builder.addIntegerProperty("LimelightCount", () -> this.getLimelights().size(), null);

        for (int idx=0; idx<this.getLimelights().size();idx++) {
            Limelight limelight = this.getLimelights().get(idx);
            String base = "Limelights/" + limelight.getName();
            builder.addDoubleProperty(base+"/aprilTag", limelight::getAprilTagID, null);
            builder.addDoubleProperty(base + "/pipeline", () ->
                            LimelightHelpers.getCurrentPipelineIndex(limelight.getName()),
                    (pidx) -> LimelightHelpers.setPipelineIndex(limelight.getName(), (int)pidx));

            VisionTypes.TargetInfo targetInfo = limelight.getTargetInfo();
            builder.addBooleanProperty(base + "/TargetInfo/hasTarget", targetInfo::hasTarget, null);
            builder.addDoubleProperty(base + "/TargetInfo/xOffset", targetInfo::xOffset, null);
            builder.addDoubleProperty(base + "/TargetInfo/yOffset", targetInfo::yOffset, null);
            builder.addDoubleProperty(base + "/TargetInfo/targetArea", targetInfo::targetArea, null);

        }
    }


    /**
     * Looks up a tag pose3d by tag ID
     * @param aprilTagID
     * @return
     */
    public Optional<Pose3d> getTagPose(int aprilTagID) {
        Optional<Pose3d> basePose = this.fieldLayout.getTagPose(aprilTagID);
        if (GameInfo.isRedAlliance() && basePose.isPresent()) {
//            this.fieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
        }
        return this.fieldLayout.getTagPose(aprilTagID);
    }
}
