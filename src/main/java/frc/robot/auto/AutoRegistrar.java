package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.strategies.*;
import frc.robot.commands.auto.*;

import com.pathplanner.lib.auto.NamedCommands;


/**
 * Helper class to register all auto strategies, named commands, and auto paths
 */
public class AutoRegistrar {

    public static void registerStrategies(AutoManager autoManager) {
        autoManager.registerStrategy("Debug Auto", "Debug Auto", DebugAutoStrategy::new, true);
        autoManager.registerStrategy("Debug Auto Path", "Debug Auto Path", DebugAutoPathStrategy::new);
        autoManager.registerStrategy("EP", "EP", EPStrategy::new);
        autoManager.registerStrategy("Spin", "Spin", EPStrategy::new);
    }

    public static void registerNamedCommands() {
         NamedCommands.registerCommand("NamedCommandPrintTest", Commands.print("Named Command Test"));
    }

    public static void registerAutoPaths(PathRegistry pathRegistry) {
        // Tab.registerAutoPaths(pathRegistry);
    }
}