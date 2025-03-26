package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.async.AsyncManager;
import frc.robot.auto.AutoManager;
import frc.robot.subsystems.Intake.FunnelSubsystem;
import frc.robot.subsystems.pose.PoseManager;
import frc.robot.subsystems.scoring.ScoreSubsystem;
import frc.robot.subsystems.*;
import frc.robot.subsystems.amd.LEDSubsystem;
import frc.robot.subsystems.Intake.AlgaeArm;
import frc.robot.subsystems.Intake.AlgaeIntake;
import frc.robot.subsystems.Intake.CoralIntake;
import frc.robot.subsystems.Prototype.JoshPrototype;
import frc.robot.subsystems.Prototype.ComponentMotor;
import frc.robot.subsystems.Prototype.PrototypeComponent;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.ArrayList;
import java.util.List;

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
    public static Elevator elevator;
    public static Climber climber;
    public static AlgaeIntake algaeIntake;
    public static AlgaeArm algaeArm;
    public static CoralIntake coralIntake;
    public static FunnelSubsystem funnelSubsystem;

    public static JoshPrototype joshPrototype;
    public static PrototypeComponent austinGearPrototype;
    public static PrototypeComponent Climberproto1;
    public static PrototypeComponent Climberproto2;

    
    public static List<Lifecycle> lifecycleSubsystems = new ArrayList<>();

    //
    // Utility classes
    //
    public static RotationController rotationController;
    public static ProfiledRotationController profiledRotationController;
    public static TranslationController translationController;
    public static PIDController alignTranslationController;
    public static AsyncManager asyncManager;
    public static AprilTagUtil aprilTagUtil;
    public static PoseManager poseManager;

    // Warning: This must be created after everything else to ensure all subsystems
    // are registered
    public static AutoManager autoManager;
    private static Subsystems instance;
    public static VisionOdometryUpdater visionOdometryUpdater;
    public static ScoreSubsystem scoreSubsystem;

    public Subsystems() {
        swerveSubsystem = Robot.robotConfig.createDrivetrain();
        visionSubsystem = new VisionSubsystem(Robot.robotConfig.getLimelights());
        ledSubsystem = new LEDSubsystem();
        elevator = new Elevator();
        climber = new Climber();
        algaeIntake = new AlgaeIntake();
        algaeArm = new AlgaeArm();
        coralIntake = new CoralIntake();
        funnelSubsystem = new FunnelSubsystem();

        // Prototype support
//        createPrototypeSubsystems();
        createUtilitySubsystems();

        // Bookkeeping and registrations
        registerLifecycleSubsystems();
        registerSmartDashboardEntries();
    }

    @SuppressWarnings("unchecked") // for inject Controls
    private void createPrototypeSubsystems() {
        joshPrototype = new JoshPrototype();
        austinGearPrototype = new ComponentMotor("austinGearPrototype", 51);

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


        //Climberproto1 = new ComponentMotor("ClimberProto3", 50, (m) -> {m.setDirection(ComponentMotor.direction.inverse);});
        //Climberproto1.InjectControls(ComponentPreconfig.ABXYpreconf);
        //Climberproto2 = new ComponentMotor( "ClimberProto4", 51, (m) -> {m.setDirection(ComponentMotor.direction.corresponding);});
    }

    private void createUtilitySubsystems() {
        rotationController = new RotationController();
        profiledRotationController = new ProfiledRotationController();
        translationController = new TranslationController();
        alignTranslationController = new PIDController(0.008, 0, 0);

        asyncManager = new AsyncManager();
        asyncManager.start();

        autoManager = new AutoManager();
        autoManager.initialize();

        aprilTagUtil = new AprilTagUtil();

        visionOdometryUpdater = new VisionOdometryUpdater(visionSubsystem, swerveSubsystem);

        scoreSubsystem = new ScoreSubsystem();

        poseManager = new PoseManager();
    }

    private void registerLifecycleSubsystems() {
        lifecycleSubsystems.add(visionSubsystem);
        lifecycleSubsystems.add(ledSubsystem);
        lifecycleSubsystems.add(elevator);
        lifecycleSubsystems.add(climber);
        lifecycleSubsystems.add(algaeIntake);
        lifecycleSubsystems.add(algaeArm);
        lifecycleSubsystems.add(coralIntake);
        lifecycleSubsystems.add(scoreSubsystem);
        lifecycleSubsystems.add(funnelSubsystem);
    }

    private void registerSmartDashboardEntries() {
//        SmartDashboard.putData("VisionOdometryUpdater", visionOdometryUpdater);
        SmartDashboard.putData("Subsystems/Elevator", elevator);
        SmartDashboard.putData("Subsystems/Climber", climber);
        SmartDashboard.putData("Subsystems/AlgaeIntake", algaeIntake);
        SmartDashboard.putData("Subsystems/AlgaeArm", algaeArm);
        SmartDashboard.putData("Subsystems/CoralIntake", coralIntake);
        SmartDashboard.putData("Subsystems/Vision", visionSubsystem);
        SmartDashboard.putData("Subsystems/Score", scoreSubsystem);
        SmartDashboard.putData("Subsystems/Funnel", funnelSubsystem);
        SmartDashboard.putData("Subsystems/AprilTagUtil", aprilTagUtil);
        SmartDashboard.putData("Subsystems/PoseManager", poseManager);

        SmartDashboard.putData("Subsystems/AsyncManager", asyncManager);

        SmartDashboard.putData("PID/RotationController", rotationController);
        SmartDashboard.putData("PID/ProfiledRotationController", profiledRotationController);
        SmartDashboard.putData("PID/TranslationController", translationController);
        SmartDashboard.putData("PID/AlignTranslationController", alignTranslationController);

        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
    }

    public static Subsystems getInstance() {
        if (instance == null) {
            instance = new Subsystems();
        }
        return instance;
    }
}
