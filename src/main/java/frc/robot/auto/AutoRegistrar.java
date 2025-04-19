package frc.robot.auto;

import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems;
import frc.robot.auto.strategies.*;
import frc.robot.auto.strategies.BayouTroisStrategy.StartingPosition;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.auto.strategies.debug.*;
import frc.robot.commands.DriveRobotCentricCommand;
import frc.robot.commands.PickAlgaeSoonerCommand;
import frc.robot.commands.auto.CenterPickHighCommand;
import frc.robot.commands.auto.CenterStartCommand;
import frc.robot.commands.vision.AlignDriveInCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake.AlgaeArm;
import frc.robot.subsystems.Intake.IntakeCoralCommand;

import javax.naming.Name;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;


/**
 * Helper class to register all auto strategies, named commands, and auto paths
 */
public class AutoRegistrar {

    public static void registerStrategies(AutoManager autoManager) {
        autoManager.registerStrategy("Debug Auto", "Debug Auto", DebugAutoStrategy::new, true);
        autoManager.registerStrategy("Debug Auto Path", "Debug Auto Path", DebugAutoPathStrategy::new);
        autoManager.registerStrategy("TestNamedAutoStrategy", "TestNamedAutoStrategy", () -> new TestNamedAutoStategy("ElevatorEvent"));

        autoManager.registerStrategy("Center", CenterAutoStrategy::new);
        autoManager.registerStrategy("Center Quals", CenterQualsAutoStrategy::new);
        autoManager.registerStrategy("Center Quals Left", CenterQualsLeftAutoStrategy::new);

        // autoManager.registerStrategy("Arkansas Right Red", "Arkansas Right Red", () -> new ArkansasStrategy(false, true));
        // autoManager.registerStrategy("Arkansas Left Red", "Arkansas Left Red", () -> new ArkansasStrategy(true, true));
//        autoManager.registerStrategy("Arkansas Right Blue", "Arkansas Right Blue", () -> new ArkansasStrategy(false, false));
        // autoManager.registerStrategy("Arkansas Left Blue", "Arkansas Left Blue", () -> new ArkansasStrategy(true, false));

        autoManager.registerStrategy("Bayou Right Red", () -> new BayouTroisStrategy(StartingPosition.RED_RIGHT));
        autoManager.registerStrategy("Bayou Left Red", () -> new BayouTroisStrategy(StartingPosition.RED_LEFT));
        autoManager.registerStrategy("Bayou Right Blue", () -> new BayouTroisStrategy(StartingPosition.BLUE_RIGHT));
        autoManager.registerStrategy("Bayou Left Blue", () -> new BayouTroisStrategy(StartingPosition.BLUE_LEFT));

//        autoManager.registerStrategy("EP", "EP", EPStrategy::new);
//        autoManager.registerStrategy("TestDP", TestDPStrategy::new);
//        autoManager.registerStrategy("Spin", "Spin", SpinStrategy::new);
//        autoManager.registerStrategy("Cresent", "Cresent", CresentStrategy::new);
//       autoManager.registerStrategy("ThreePiece", "ThreePiece", ThreePieceStrategy::new);
//        autoManager.registerStrategy("BlueReefCheck", "BlueReefCheck", () -> new SimplePathStrategy("BlueReefCheck"));
    }

    public static void registerEventTriggers() {
        new EventTrigger("triggerElevL4").onTrue(
                Subsystems.elevator.slowMoveToPositionCommand(Elevator.ElevatorSetpoint.L4));
        new EventTrigger("triggerElevL4v2").onTrue(
                Subsystems.elevator.slowMoveToPositionCommand(Elevator.ElevatorSetpoint.L4));

        new EventTrigger("debugEvent").onTrue(new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L4).withNoWait());
    }

    public static void registerNamedCommands() {
         NamedCommands.registerCommand("NamedCommandPrintTest", Commands.print("Named Command Test"));
         NamedCommands.registerCommand("intakeCoral", new IntakeCoralCommand().withTimeout(5.0));

         // Center Auto
        NamedCommands.registerCommand("centerStartCommand", new CenterStartCommand());
        NamedCommands.registerCommand("centerPickHigh", new CenterPickHighCommand());

        NamedCommands.registerCommand("alignDriveRight", new AlignDriveInCommand(AlignDriveInCommand.AlignTarget.RIGHT));
        NamedCommands.registerCommand("elevatorZero", new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.Zero));

        NamedCommands.registerCommand("elevatorReefHigh", new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.AlgaeReefHigh).withTimeout(1.0));
        NamedCommands.registerCommand("elevatorReefLow", new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.AlgaeReefLow).withTimeout(1.0));
        NamedCommands.registerCommand("elevatorReefHighNoWait", new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.AlgaeReefHigh).withNoWait());
        NamedCommands.registerCommand("elevatorReefLowNoWait", new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.AlgaeReefLow).withNoWait());
        NamedCommands.registerCommand("elevatorL4", new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L4));
        NamedCommands.registerCommand("elevatorL4NoWait", new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L4).withNoWait());


//        NamedCommands.registerCommand("shootCoral", Subsystems.coralIntake.shootCoralCommand().withTimeout(0.5));
//        NamedCommands.registerCommand("stopCoral", Subsystems.coralIntake.stopCommand());
        NamedCommands.registerCommand("pickAlgaeHigh", new PickAlgaeSoonerCommand.AutoPick(Elevator.ElevatorSetpoint.AlgaeReefHigh));
        NamedCommands.registerCommand("pickAlgaeLow", new PickAlgaeSoonerCommand.AutoPick(Elevator.ElevatorSetpoint.AlgaeReefLow));
        NamedCommands.registerCommand("holdAlgae", Subsystems.algaeIntake.holdAlgaeROCommand());

        NamedCommands.registerCommand("scoreCoralL4", Commands.sequence(
                new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L4).withTimeout(1.5),
                Subsystems.coralIntake.shootCoralCommand().withTimeout(0.5),
                Subsystems.coralIntake.stopCommand()
                ));

        NamedCommands.registerCommand("scoreAlgaeInBarge", Commands.sequence(
                new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L4).withTimeout(1),
                Subsystems.algaeIntake.ejectCommand().withTimeout(0.5),
                Subsystems.algaeIntake.stopCommand().withTimeout(0.1)
                ));

        NamedCommands.registerCommand("backOffBarge",
                new DriveRobotCentricCommand(Seconds.of(0.5))
                        .withApproachSpeed(MetersPerSecond.of(-0.5)).withTimeout(0.5));
    }


    public static void registerAutoPaths(PathRegistry pathRegistry) {
    }
}