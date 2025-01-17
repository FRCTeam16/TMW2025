package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

public class VisionTypes {

    /**
     * LED modes
     */
    public enum LEDMode {
        CurrentPipeline(0),
        ForceOff(1),
        ForceBlink(2),
        ForceOn(3);

        private final int mode;

        LEDMode(int mode) {
            this.mode = mode;
        }
    }

    public record CameraDistanceValues(Distance heightToCamera, Distance heightToTarget, Angle cameraAngle) {}

    public record TargetInfo(boolean hasTarget, double xOffset, double yOffset, double latency, VisionTypes.CameraDistanceValues cameraDistanceValues) {

        public Distance calculateDistance() {
            return calculateDistance(this.cameraDistanceValues.heightToTarget);
        }

        public Distance calculateDistance(Distance heightToTarget) {
            if (this.hasTarget) {
                double goalRadians = this.cameraDistanceValues.cameraAngle.plus(Degrees.of(this.yOffset)).in(Radians);
                // TODO: Unit test this
                return heightToTarget.minus(this.cameraDistanceValues.heightToCamera).div(Math.tan(goalRadians));
//                return (heightToTarget.in(Inches) - this.cameraDistanceValues.heightToCamera.in(Inches)) / Math.tan(goalRadians);
            } else {
                return Inches.of(-1);
            }
        }
    }

    public record PoseInfo(Pose2d pose, double latency) {
    }

    public record LimelightInfo(String name, Distance heightToCamera, Angle cameraAngle) {}
}
