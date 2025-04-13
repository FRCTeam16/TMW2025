package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.util.BSLogger;

import java.util.HashMap;
import java.util.function.Supplier;

/**
 * Manages the autonomous mode of the robot.
 * <p>
 * This class is responsible for registering and selecting autonomous strategies,
 * named commands, and auto paths. It uses a SendableChooser to allow the user
 * to select the desired autonomous strategy from the SmartDashboard.
 */
public class AutoManager {

    private static final PathRegistry pathRegistry = new PathRegistry();
    private final SendableChooser<String> chooser = new SendableChooser<>();
    private final HashMap<String, Command> strategyLookup = new HashMap<>();

    /**
     * Constructs an AutoManager instance.
     */
    public AutoManager() {
    }

    /**
     * Initializes the autonomous manager.
     * Must be called before using the manager.
     * Registers named commands, auto paths, and strategies.
     */
    public void initialize() {
        AutoRegistrar.registerEventTriggers();
        AutoRegistrar.registerNamedCommands();
        AutoRegistrar.registerAutoPaths(pathRegistry);
        AutoRegistrar.registerStrategies(this);

        SmartDashboard.putData("Auto Selector", chooser);
    }

    /**
     * Returns the command for the given path name.
     *
     * @param pathName the name of the path
     * @return the command for the path
     */
    public Command getAutoPath(String pathName) {
        return pathRegistry.getPath(pathName);
    }

    public Command followPathCommand(String pathName) {
        try {
            // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

            // Create a path following command using AutoBuilder. This will also trigger event markers.
            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            BSLogger.log("AutoManager", "Error getting auto path: " + e.getMessage());
            return Commands.none();
        }
    }

    public Command pathfindThenFollowPathCommand(String pathName) {
        try {
            // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            // Create a path following command using AutoBuilder. This will also trigger event markers.
            return AutoBuilder.pathfindThenFollowPath(path, Constants.pathConstraints);
        } catch (Exception e) {
            BSLogger.log("AutoManager", "Error getting auto path: " + e.getMessage());
            return Commands.none();
        }
    }

    /**
     * Registers an autonomous strategy with the given name and supplier.
     *
     * @param strategyName the name of the strategy
     * @param strategy     the supplier of the command for the strategy
     */
    public void registerStrategy(String strategyName, Supplier<Command> strategy) {
        registerStrategy(strategyName, strategyName, strategy, false);
    }

    /**
     * Registers an autonomous strategy with the given display name, strategy name, and supplier.
     *
     * @param displayName  the display name of the strategy
     * @param strategyName the name of the strategy
     * @param strategy     the supplier of the command for the strategy
     */
    public void registerStrategy(String displayName, String strategyName, Supplier<Command> strategy) {
        registerStrategy(displayName, strategyName, strategy, false);
    }

    /**
     * Registers an autonomous strategy with the given display name, strategy name, supplier, and default flag.
     *
     * @param displayName  the display name of the strategy
     * @param strategyName the name of the strategy
     * @param strategy     the supplier of the command for the strategy
     * @param isDefault    whether the strategy should be the default option
     */
    public void registerStrategy(String displayName, String strategyName, Supplier<Command> strategy, boolean isDefault) {
        if (isDefault) {
            chooser.setDefaultOption(displayName, strategyName);
        } else {
            chooser.addOption(displayName, strategyName);
        }
        strategyLookup.put(strategyName, strategy.get());
    }

    /**
     * Returns the currently selected autonomous strategy.
     *
     * @return the currently selected autonomous strategy
     */
    public Command getSelectedAutoStrategy() {
        Command selected = null;

        if (strategyLookup.containsKey(chooser.getSelected())) {
            selected = strategyLookup.get(chooser.getSelected());
        } else {
            selected = Commands.print("[AutoManager] ERROR: Could not find requested auto: " + chooser.getSelected());
        }
        return selected;
    }

    /**
     * Displays the currently selected autonomous strategy on the SmartDashboard.
     */
    public void showSelectedAuto() {
        var selected = chooser.getSelected();
        SmartDashboard.putString("Selected Auto", (selected != null) ? selected : "Unknown");
    }
}