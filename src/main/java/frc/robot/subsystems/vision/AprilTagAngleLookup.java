package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Subsystems;

import java.util.HashMap;
import java.util.Optional;

public class AprilTagAngleLookup {
    private static final HashMap<Integer, Double> tagFacingAngles = new HashMap<>();

    static {
        // Speaker Tags
        tagFacingAngles.put(17, 45.0);
        tagFacingAngles.put(18, 0.0);
        tagFacingAngles.put(19, 315.0);
        tagFacingAngles.put(20, 225.0);
        tagFacingAngles.put(21, 180.0);
        tagFacingAngles.put(22, 135.0);

        // Amp Tags
        tagFacingAngles.put(11, 45.0);
        tagFacingAngles.put(10, 0.0);
        tagFacingAngles.put(9, 315.0);
        tagFacingAngles.put(8, 225.0);
        tagFacingAngles.put(7, 180.0);
        tagFacingAngles.put(6, 135.0);

        // Stage Tags
        tagFacingAngles.put(12, 225.0);
        tagFacingAngles.put(13, 135.0);
        tagFacingAngles.put(14, 0.0);
        tagFacingAngles.put(15, 0.0);
        tagFacingAngles.put(16, 270.0);

        // Source Tags
        tagFacingAngles.put(4, 180.0);
        tagFacingAngles.put(5, 180.0);
        tagFacingAngles.put(3, 90.0);
        tagFacingAngles.put(2, 45.0);
        tagFacingAngles.put(1, 315.0);
    }

    public static Optional<Angle> getFacingAngle(int aprilTagId) {
        Double angle = tagFacingAngles.get(aprilTagId);
        return angle == null ? Optional.empty() : Optional.of(Units.Degrees.of(angle));
    }

    public static Optional<Double> getPerpendicularAngle(int aprilTagId) {
        var tagPose = Subsystems.visionSubsystem.getTagPose(aprilTagId);
        if (tagPose.isEmpty()) {
            return Optional.empty();
        }

        Rotation2d rotation = tagPose.get().getRotation().toRotation2d();
        // Add 90 degrees to get perpendicular angle
        double perpendicular = rotation.getDegrees() + 90.0;
        // Normalize to 0-360 range
        perpendicular = perpendicular % 360.0;
        if (perpendicular < 0) {
            perpendicular += 360.0;
        }
        return Optional.of(perpendicular);
    }
}