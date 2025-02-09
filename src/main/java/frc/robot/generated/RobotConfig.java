package frc.robot.generated;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.VisionTypes;
import frc.robot.util.BSLogger;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Collections;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

/**
 * The `RobotConfig` class is a singleton that manages the configuration of the robot.
 * It determines which set of constants to use based on an environment variable or configuration file.
 */
public class RobotConfig {
    private static RobotConfig instance;
    private final ConfigName config;
    private final CanIDLookup canIDLookup;


    /**
     * Private constructor to prevent instantiation.
     * Initializes the configuration based on the environment variable or configuration file.
     */
    private RobotConfig() {
        String potentialConfigName = lookupConfiguredName();
        this.config = switch (potentialConfigName.toLowerCase()) {
            case "lowrida" -> ConfigName.LOWRIDA;
            default -> ConfigName.DEFAULT;
        };
        this.canIDLookup = new CanIDLookup(this.config);
        BSLogger.log("RobotConfig", "Using configuration: " + this.config);
    }

    /**
     * Returns the singleton instance of the `RobotConfig` class.
     *
     * @return the singleton instance of `RobotConfig`
     */
    public static RobotConfig getInstance() {
        if (instance == null) {
            instance = new RobotConfig();
        }
        return instance;
    }

    public Iterable<Limelight> getLimelights() {
        return switch (this.config) {
            case LOWRIDA -> Stream.of(
//                    new VisionTypes.LimelightInfo("limelight", Inches.of(6), Degrees.of(26.84)),
                            new VisionTypes.LimelightInfo("limelight-lfour", Inches.of(6), Degrees.of(26.84)),
                            new VisionTypes.LimelightInfo("limelight-right", Inches.of(6), Degrees.of(-26.84)))
                    .map(Limelight::new).collect(Collectors.toSet());
            default -> Collections.emptySet();
        };
    }

    /**
     * Returns the configuration name.
     *
     * @return the configuration name
     */
    public ConfigName getConfigName() {
        return config;
    }

    /**
     * Determines the configuration name by checking an environment variable,
     * reading a file from a specific path, or using a default value.
     *
     * @return the configuration name
     */
    private String lookupConfiguredName() {
        // Check if the environment variable is set
        String configName = System.getenv("TMW2025CONFIG");
        if (configName != null && !configName.isEmpty()) {
            return configName;
        }

        // Try to read the file from "/home/lvuser/2025-robotconfig.txt"
        try {
            Path path = Paths.get("/home/lvuser/2025-robotconfig.txt");
            if (Files.exists(path)) {
                return Files.readString(path).trim();
            }
        } catch (IOException e) {
            System.err.println("Error reading /home/lvuser/2025-robotconfig.txt: " + e.getMessage());
        }

        // Try to read the file from the deploy folder
        try {
            Path path = Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "2025-robotconfig.txt");
            if (Files.exists(path)) {
                return Files.readString(path).trim();
            }
        } catch (IOException e) {
            System.err.println("Error reading deploy/2025-robotconfig.txt: " + e.getMessage());
        }

        // Default value if none of the above methods work
        return "default";
    }

    /**
     * Creates a `CommandSwerveDrivetrain` instance based on the current configuration.
     *
     * @return a `CommandSwerveDrivetrain` instance
     */
    public CommandSwerveDrivetrain createDrivetrain() {
        return switch (this.config) {
            case LOWRIDA -> LowridaTunerConstants.createDrivetrain();
            default -> TunerConstants.createDrivetrain();
        };
    }

    public int getCanID(String motorName) {
        return canIDLookup.getCanID(motorName);
    }

    /**
     * Enum representing the possible configurations.
     */
    public enum ConfigName {
        DEFAULT, LOWRIDA
    }
}