package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionUtil {
    private final NetworkTable limelightTable;


    /**
     * Constructs the utility class, connecting to the Limelight's NetworkTable.
     * @param limelightName The name of the Limelight network table (e.g., "limelight").
     */
    public VisionUtil(String limelightName) {
        limelightTable = NetworkTableInstance.getDefault().getTable(limelightName);
    }

    /**
     * Checks if the Limelight has a valid AprilTag target.
     * @return true if a target is detected, false otherwise.
     */
    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0) > 0.0;
    }

    /**
     * Gets the ID of the detected AprilTag.
     * @return The tag ID, or -1 if no target is detected.
     */
    public int getTagID() {
        return hasTarget() ? (int) limelightTable.getEntry("tid").getDouble(-1) : -1;
    }

    /**
     * Gets the horizontal offset (tx) of the AprilTag from the center of the camera's view.
     * @return The horizontal angle in degrees, or 0.0 if no target is detected.
     */
    public double getHorizontalOffset() {
        return hasTarget() ? limelightTable.getEntry("tx").getDouble(0.0) : 0.0;
    }

    /**
     * Gets the vertical offset (ty) of the AprilTag from the center of the camera's view.
     * @return The vertical angle in degrees, or 0.0 if no target is detected.
     */
    public double getVerticalOffset() {
        return hasTarget() ? limelightTable.getEntry("ty").getDouble(0.0) : 0.0;
    }

    /**
     * Calculates the angle to the AprilTag in the camera's plane.
     * @return The angle in degrees from the camera's horizontal axis, or 0.0 if no target is detected.
     */
    public double calculateAngleToTag() {
        if (!hasTarget()) return 0.0;

        double tx = getHorizontalOffset();
        double ty = getVerticalOffset();

        // Use Pythagorean theorem to find the resultant angle.
        return Math.toDegrees(Math.atan2(ty, tx));
    }

    /**
     * Gets the distance to the detected AprilTag based on the tag's size in the camera's view.
     * Requires configuration based on field measurements and camera parameters.
     * @param targetHeightMeters The known height of the target in meters.
     * @param cameraHeightMeters The height of the camera in meters.
     * @param cameraAngleDegrees The mounting angle of the camera in degrees.
     * @return The distance to the AprilTag in meters, or 0.0 if no target is detected.
     */
    public double getDistanceToTag(double targetHeightMeters, double cameraHeightMeters, double cameraAngleDegrees) {
        if (!hasTarget()) return 0.0;

        double ty = getVerticalOffset();
        double angleToTarget = Math.toRadians(cameraAngleDegrees + ty);

        // Calculate distance using trigonometry
        return (targetHeightMeters - cameraHeightMeters) / Math.tan(angleToTarget);
    }
}
