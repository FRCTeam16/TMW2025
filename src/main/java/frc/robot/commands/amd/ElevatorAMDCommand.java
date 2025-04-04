package frc.robot.commands.amd;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.amd.AMDSerialData;
import frc.robot.subsystems.amd.AMDStats;
import frc.robot.subsystems.amd.AbstractDataCollector;

import java.util.List;

public class ElevatorAMDCommand extends Command {
    private final ElevatorDataCollector dataCollector = new ElevatorDataCollector();
    private Command amdCommand;

    public ElevatorAMDCommand() {
        // No requirements needed - internal commands will handle them
    }

    @Override
    public void initialize() {
        amdCommand = createAMDCommand();
        amdCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return !amdCommand.isScheduled();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            amdCommand.cancel();
        }
    }

    private Command createAMDCommand() {
        return Commands.sequence(
                        // Phase 1
                        Commands.parallel(
                                        new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L2),
                                        collectDataCommand().repeatedly()
                                )
                                .until(() -> Subsystems.elevator.isInPosition())
                                .withTimeout(2)
                                .handleInterrupt(() -> dataCollector.setTimedOut(true)),

                        // Phase 2
                        Commands.parallel(
                                        collectDataCommand().repeatedly()
                                )
                                .withTimeout(2.0),

                        // Phase 3
                        Commands.parallel(
                                        new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.Zero),
                                        collectDataCommand().repeatedly()
                                )
                                .until(() -> Subsystems.elevator.isInPosition())
                                .withTimeout(2.0)
                                .handleInterrupt(() -> dataCollector.setTimedOut(true))
                )
                .beforeStarting(() ->
                        Subsystems.ledSubsystem.getAMDSerialData().startAMDPhase(AMDSerialData.AMDPhase.Elevator)
                )
                .finallyDo(() -> {
                    dataCollector.report();
                });
    }

    private Command collectDataCommand() {
        return Commands.runOnce(() -> {
            Subsystems.elevator.collectAMDData(dataCollector);
            dataCollector.report();
        });
    }

    public static class ElevatorDataCollector extends AbstractDataCollector<Pair<Integer, Integer>> {
        List<Boolean> inPosition = new java.util.ArrayList<>();
        List<Double> leftCurrents = new java.util.ArrayList<>();
        List<Double> rightCurrents = new java.util.ArrayList<>();
        private boolean timedOut;

        public void collectData(boolean inPosition, StatusSignal<Current> statorCurrent, StatusSignal<Current> current) {
            leftCurrents.add(statorCurrent.getValueAsDouble());
            rightCurrents.add(statorCurrent.getValueAsDouble());
        }

        @Override
        public Pair<Integer, Integer> getScore() {
            if (timedOut) {
                return new Pair<>(1,2);
            }
            List<Double> leftScore = AMDStats.detectOutliersZScore(leftCurrents, 70);
            List<Double> rightScore = AMDStats.detectOutliersZScore(rightCurrents, 70);
            int leftCount = leftScore.isEmpty() ? 1 : 2;
            int rightCount = rightScore.isEmpty() ? 1 : 2;
            return new Pair<>(leftCount, rightCount);
        }

        public void setTimedOut(boolean b) {
            this.timedOut = true;
        }

        public void report() {
            Pair<Integer, Integer> score = getScore();
            Subsystems.ledSubsystem.getAMDSerialData().submitElevatorScore(score.getFirst(), score.getSecond());
        }
    }
}