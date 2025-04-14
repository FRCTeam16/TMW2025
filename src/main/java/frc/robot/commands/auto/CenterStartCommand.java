package frc.robot.commands.auto;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Subsystems;
import frc.robot.commands.DriveRobotCentricCommand;
import frc.robot.commands.PickAlgaeSoonerCommand;
import frc.robot.commands.vision.AlignDriveInCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake.AlgaeArm;
import frc.robot.subsystems.vision.Pipeline;

import static edu.wpi.first.units.Units.MetersPerSecond;
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
                        new AlignDriveInCommand(AlignDriveInCommand.AlignTarget.RIGHT).withResetPoseDuringDrive(false)
                ),
                new SequentialCommandGroup(
                        Subsystems.algaeIntake.ejectCommand().withTimeout(0.3),
                        Subsystems.algaeIntake.stopCommand().withTimeout(0.1)
                ),
                Commands.parallel(
                        new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.AlgaeReefLow).withSlowMode(),
                        Commands.runOnce(() -> Subsystems.algaeArm.setArmPosition(AlgaeArm.AlgaeArmPosition.PickFromReef)),
                        Commands.runOnce(() -> Subsystems.algaeIntake.intakeAlgae())
                ),
                new WaitCommand(1.5).until(Subsystems.algaeIntake::isAlgaeDetected),
                Subsystems.algaeArm.setArmPositionCommand(AlgaeArm.AlgaeArmPosition.Up).withTimeout(0.1),
                Subsystems.algaeIntake.holdAlgaeROCommand()
        );
    }

}
