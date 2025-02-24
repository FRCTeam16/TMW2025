package frc.robot.subsystems.vision;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

import java.util.HashMap;
import java.util.Optional;

/**
 * Class to lookup the facing angle of an AprilTag. Angles manually entered.
 */
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
}