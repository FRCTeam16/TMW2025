package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.async.AsyncManager;
import frc.robot.auto.AutoManager;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Lifecycle;
import frc.robot.subsystems.RotationController;
import frc.robot.subsystems.DMS.LEDSubsystem;
import frc.robot.subsystems.VisionOdometryUpdater;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionTypes;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

/**
 * The Subsystems class represents a collection of subsystems used in the robot.
 * It provides singleton access to all the subsystem instances and manages their
 * lifecycle.
 */
@SuppressWarnings("InstantiationOfUtilityClass")
public class Subsystems {
    public static CommandSwerveDrivetrain swerveSubsystem;
    public static VisionSubsystem visionSubsystem;
    public static LEDSubsystem ledSubsystem;

    public static List<Lifecycle> lifecycleSubsystems = new ArrayList<>();

    //
    // Utility classes
    //
    public static RotationController rotationController;
    public static AsyncManager asyncManager;

    // Warning: This must be created after everything else to ensure all subsystems
    // are registered
    public static AutoManager autoManager;
    private static Subsystems instance;
    public static VisionOdometryUpdater visionOdometryUpdater;

    public Subsystems() {
        swerveSubsystem = TunerConstants.createDrivetrain();
        visionSubsystem = new VisionSubsystem(
                Stream.of(
                    new VisionTypes.LimelightInfo("limelight", Inches.of(6), Degrees.of(26.84)),
                    new VisionTypes.LimelightInfo("limelight-right", Inches.of(6), Degrees.of(0)))
                .map(Limelight::new).collect(Collectors.toSet()));
        ledSubsystem = new LEDSubsystem();

        lifecycleSubsystems.add(visionSubsystem);
        lifecycleSubsystems.add(ledSubsystem);

        // SmartDashboard.putData("ShooterSubsystem", shooter);
        createUtilitySubsystems();
    }

    private void createUtilitySubsystems() {
        rotationController = new RotationController();

        asyncManager = new AsyncManager();
        asyncManager.start();

        autoManager = new AutoManager();
        autoManager.initialize();

        visionOdometryUpdater = new VisionOdometryUpdater(visionSubsystem, swerveSubsystem);
    }

    public static Subsystems getInstance() {
        if (instance == null) {
            instance = new Subsystems();
        }
        return instance;
    }
}
