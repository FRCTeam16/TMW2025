package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.DegreesPerSecond;

public class ProfiledRotationController extends ProfiledPIDController {
    private static final double maxDegreesPerSecond = Constants.MaxAngularRate.in(DegreesPerSecond); // FIXME Lookup Math.toDegrees(Constants.Swerve.kMaxAngularVelocity) / 1.5;
    private static final double SPEED_CLAMP = 0.2;

    private double kP = 0.06; // 13.0; //0.01; //4.25;
    private double kI = 0.0; // 3.0;
    private double kD = 0.04; // 0.5; // 0;

    private double tolerance = 2.0;

    public ProfiledRotationController() {
        super(1.6, 0.0, 0.04, new TrapezoidProfile.Constraints(maxDegreesPerSecond, maxDegreesPerSecond * 2));
    }

    public ProfiledRotationController(double kp, double ki, double kd) {
        super(kp, ki, kd, new TrapezoidProfile.Constraints(maxDegreesPerSecond, maxDegreesPerSecond * 2));
        this.enableContinuousInput(-180, 180);
        this.setIntegratorRange(-5, 5);
        this.setTolerance(tolerance);
    }

    public static double clampToDPS(double outputPercent) {
        return MathUtil.clamp(outputPercent, -SPEED_CLAMP, SPEED_CLAMP) * maxDegreesPerSecond;
    }

    public void resetTolerance() {
        this.setTolerance(tolerance);
    }
}
