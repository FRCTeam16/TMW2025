package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.async.AsyncManager;
import frc.robot.auto.AutoManager;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DMS.LEDSubsystem;
import frc.robot.subsystems.Intake.AlgaeIntake;
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
    public static Elevator elevator;
    public static Climber climber;
    public static PrototypeGenericMotor Climberproto1;
    public static PrototypeGenericMotor Climberproto2;
    public static AlgaeIntake algaeIntake;
    
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
        swerveSubsystem = Robot.robotConfig.createDrivetrain();
        visionSubsystem = new VisionSubsystem(Robot.robotConfig.getLimelights());
        ledSubsystem = new LEDSubsystem();
        joshPrototype = new JoshPrototype();
        austinGearPrototype = new PrototypeGenericMotor("austinGearPrototype", 51);
        elevator = new Elevator();
        algaeIntake = new AlgaeIntake();



//         fourMotorElevator = new ComposedPrototype( (components) -> {
//                PrototypeGeneric.filterByType(components, PrototypeGenericMotor.class).get(0).setDirection(PrototypeGenericMotor.direction.corresponding);
//                PrototypeGeneric.filterByType(components, PrototypeGenericMotor.class).get(1).setDirection(PrototypeGenericMotor.direction.corresponding);
//                PrototypeGeneric.filterByType(components, PrototypeGenericMotor.class).get(2).setDirection(PrototypeGenericMotor.direction.inverse);
//                PrototypeGeneric.filterByType(components, PrototypeGenericMotor.class).get(3).setDirection(PrototypeGenericMotor.direction.inverse);
//            },
//            new PrototypeGenericMotor("ElevatorMotor1", 60),
//            new PrototypeGenericMotor("ElevatorMotor2", 61),
//            new PrototypeGenericMotor("ElevatorMotor3", 62),
//            new PrototypeGenericMotor("ElevatorMotor4", 63)
//        );


        Climberproto1 = new PrototypeGenericMotor("ClimberProto3", 50, (m) -> {m.setDirection(PrototypeGenericMotor.direction.inverse);});
        Climberproto2 = new PrototypeGenericMotor("ClimberProto4", 51, (m) -> {m.setDirection(PrototypeGenericMotor.direction.corresponding);});

        lifecycleSubsystems.add(visionSubsystem);
        lifecycleSubsystems.add(ledSubsystem);

        createUtilitySubsystems();

        // Add SD registrations
        registerSmartDashboardEntries();
    }

    private void createUtilitySubsystems() {
        rotationController = new RotationController();

        asyncManager = new AsyncManager();
        asyncManager.start();

        autoManager = new AutoManager();
        autoManager.initialize();

        visionOdometryUpdater = new VisionOdometryUpdater(visionSubsystem, swerveSubsystem);
    }

    private void registerSmartDashboardEntries() {
        SmartDashboard.putData("VisionOdometryUpdater", visionOdometryUpdater);
        SmartDashboard.putData("AlgaeIntake", algaeIntake);
    }

    public static Subsystems getInstance() {
        if (instance == null) {
            instance = new Subsystems();
        }
        return instance;
    }
}
