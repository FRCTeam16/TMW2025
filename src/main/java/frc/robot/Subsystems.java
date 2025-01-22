package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import frc.robot.async.AsyncManager;
import frc.robot.auto.AutoManager;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DMS.LEDSubsystem;
import frc.robot.subsystems.Prototype.ComposedPrototype;
import frc.robot.subsystems.Prototype.JoshPrototype;
import frc.robot.subsystems.Prototype.PrototypeGenericMotor;
import frc.robot.subsystems.Prototype.PrototypeGeneric;
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
    public static JoshPrototype joshPrototype;
    public static PrototypeGenericMotor austinGearPrototype;
    public static ComposedPrototype fourMotorElevator;

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
        joshPrototype = new JoshPrototype();
        austinGearPrototype = new PrototypeGenericMotor("austinGearPrototype", 51);

         fourMotorElevator = new ComposedPrototype(
            new PrototypeGenericMotor("ElevatorMotor1", 60),
            new PrototypeGenericMotor("ElevatorMotor2", 61),
            new PrototypeGenericMotor("ElevatorMotor3", 62),
            new PrototypeGenericMotor("ElevatorMotor4", 63)
        );

        lifecycleSubsystems.add(visionSubsystem);
        lifecycleSubsystems.add(ledSubsystem);

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
