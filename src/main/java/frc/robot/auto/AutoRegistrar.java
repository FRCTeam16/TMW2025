package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.strategies.*;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.auto.strategies.debug.*;
import frc.robot.subsystems.Intake.IntakeCoralCommand;


/**
 * Helper class to register all auto strategies, named commands, and auto paths
 */
public class AutoRegistrar {

    public static void registerStrategies(AutoManager autoManager) {
        autoManager.registerStrategy("Debug Auto", "Debug Auto", DebugAutoStrategy::new, true);
        autoManager.registerStrategy("Debug Auto Path", "Debug Auto Path", DebugAutoPathStrategy::new);
        autoManager.registerStrategy("Arkansas Left", "Arkansas Left", () -> new ArkansasStrategy(true));
        autoManager.registerStrategy("Arkansas Right", "Arkansas Right", () -> new ArkansasStrategy(false));
        autoManager.registerStrategy("EP", "EP", EPStrategy::new);
//        autoManager.registerStrategy("Spin", "Spin", SpinStrategy::new);
//        autoManager.registerStrategy("Cresent", "Cresent", CresentStrategy::new);
//        autoManager.registerStrategy("ThreePiece", "ThreePiece", ThreePieceStrategy::new);
//        autoManager.registerStrategy("BlueReefCheck", "BlueReefCheck", () -> new SimplePathStrategy("BlueReefCheck"));
    }

    public static void registerNamedCommands() {
         NamedCommands.registerCommand("NamedCommandPrintTest", Commands.print("Named Command Test"));
         NamedCommands.registerCommand("IntakeCoral", new IntakeCoralCommand());
    }

    public static void registerAutoPaths(PathRegistry pathRegistry) {
        // Tab.registerAutoPaths(pathRegistry);
    }
}