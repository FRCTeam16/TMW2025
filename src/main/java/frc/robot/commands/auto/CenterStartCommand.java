package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems;
import frc.robot.commands.DriveRobotCentricCommand;
import frc.robot.commands.PickAlgaeSoonerCommand;
import frc.robot.commands.vision.AlignDriveInCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake.AlgaeArm;
import frc.robot.subsystems.vision.Pipeline;

import static edu.wpi.first.units.Units.Seconds;

/**
 * Command group for the center start position in the auto routine.
 * Currently implemented in the CenterAuto in PathPlanner.
 */
public class CenterStartCommand extends SequentialCommandGroup {
    public CenterStartCommand() {
        addCommands(
                new ParallelCommandGroup(
                        Commands.runOnce(() -> Subsystems.visionSubsystem.selectPipeline(Pipeline.April)),
                        new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L4).withNoWait(),
                        new DriveRobotCentricCommand(Seconds.of(0.6))
                ),
                new ParallelDeadlineGroup(
                        new WaitCommand(1.5),
                        new AlignDriveInCommand(AlignDriveInCommand.AlignTarget.RIGHT)
                ),
                new SequentialCommandGroup(
//                        new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L4).withTimeout(1.3),
                        Subsystems.algaeIntake.ejectCommand().withTimeout(0.3),
                        Subsystems.algaeIntake.stopCommand().withTimeout(0.1)

                ),
//                new ParallelDeadlineGroup(
//                        new WaitCommand(2.5),
//                        new PickAlgaeSoonerCommand(Elevator.ElevatorSetpoint.AlgaeReefLow)
//                ),
                new PickAlgaeSoonerCommand.AutoPick(Elevator.ElevatorSetpoint.AlgaeReefLow).withTimeout(2.0),
                Subsystems.algaeArm.setArmPositionCommand(AlgaeArm.AlgaeArmPosition.Up).withTimeout(0.25),
                Subsystems.algaeIntake.holdAlgaeROCommand(),
                new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L4).withNoWait()
        );
    }

}
