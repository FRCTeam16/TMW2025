package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class Constants {
    public static final LinearVelocity MaxSpeed = MetersPerSecond.of(4.402);
    public static final AngularVelocity MaxAngularRate = RotationsPerSecond.of(1.25);

    /**
     * Pathfinding constraints for the robot.
     */
    public static PathConstraints pathConstraints =
            new PathConstraints(2.5, 2.5, 1.0, 1.0);

    public static class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 4.4;
        public static final double kMaxAccelerationMetersPerSecondSquared = 8;
        public static final double kDriveP = 2;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;
    }


    public static class DebugSendables {
        public static boolean AlgaeArm = true;
        public static boolean AlgaeIntake = true;
        public static boolean AprilTagUtil = false;
        public static boolean Climber = false;
        public static boolean CoralIntake = true;
        public static boolean Elevator = true;
        public static boolean Funnel = true;
    }
}
