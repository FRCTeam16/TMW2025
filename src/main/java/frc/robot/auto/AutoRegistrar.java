package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems;
import frc.robot.auto.strategies.*;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.auto.strategies.debug.*;


/**
 * Helper class to register all auto strategies, named commands, and auto paths
 */
public class AutoRegistrar {

    public static void registerStrategies(AutoManager autoManager) {
        autoManager.registerStrategy("Debug Auto", "Debug Auto", DebugAutoStrategy::new, true);
        autoManager.registerStrategy("Debug Auto Path", "Debug Auto Path", DebugAutoPathStrategy::new);
        autoManager.registerStrategy("Arkansas Right Red", "Arkansas Right Red", () -> new ArkansasStrategy(false, true));
        autoManager.registerStrategy("Arkansas Left Red", "Arkansas Left Red", () -> new ArkansasStrategy(true, true));
        autoManager.registerStrategy("Arkansas Right Blue", "Arkansas Right Blue", () -> new ArkansasStrategy(false, false));
        autoManager.registerStrategy("Arkansas Left Blue", "Arkansas Left Blue", () -> new ArkansasStrategy(true, false));
//        autoManager.registerStrategy("EP", "EP", EPStrategy::new);
//        autoManager.registerStrategy("TestDP", TestDPStrategy::new);
//        autoManager.registerStrategy("Spin", "Spin", SpinStrategy::new);
//        autoManager.registerStrategy("Cresent", "Cresent", CresentStrategy::new);
//        autoManager.registerStrategy("ThreePiece", "ThreePiece", ThreePieceStrategy::new);
//        autoManager.registerStrategy("BlueReefCheck", "BlueReefCheck", () -> new SimplePathStrategy("BlueReefCheck"));
    }

    public static void registerNamedCommands() {
         NamedCommands.registerCommand("NamedCommandPrintTest", Commands.print("Named Command Test"));
    }

    public static void registerAutoPaths(PathRegistry pathRegistry) {
        // Tab.registerAutoPaths(pathRegistry);
    }
}