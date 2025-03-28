package frc.robot.commands.amd;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.amd.AMDSerialData;
import frc.robot.subsystems.amd.AMDStats;
import frc.robot.subsystems.amd.AbstractDataCollector;

import java.util.List;


/**
 * TODO: run to l2/l3, pause, then home with timeout
 * send
 */
public class ElevatorAMDCommand extends Command {
    private ElevatorDataCollector dataCollector = new ElevatorDataCollector();
    private Timer timer = new Timer();
    private Elevator.ElevatorMoveToPositionCommand moveElevatorCommand;

    public ElevatorAMDCommand() {
        addRequirements(Subsystems.elevator);
    }

    @Override
    public void initialize() {
        Subsystems.ledSubsystem.getAMDSerialData().startAMDPhase(AMDSerialData.AMDPhase.Elevator);
        timer.reset();
        moveElevatorCommand = new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L4);
        moveElevatorCommand.schedule();
    }

    @Override
    public void execute() {
        Subsystems.elevator.collectAMDData(dataCollector);
    }

    @Override
    public boolean isFinished() {
        return moveElevatorCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        Pair<Integer, Integer> score = dataCollector.getScore();
        Subsystems.ledSubsystem.getAMDSerialData().submitElevatorScore(score.getFirst(), score.getSecond());
        Subsystems.ledSubsystem.getAMDSerialData().startAMDPhase(AMDSerialData.AMDPhase.Comm);
        moveElevatorCommand.cancel();
    }

    public static class ElevatorDataCollector extends AbstractDataCollector<Pair<Integer, Integer>> {
        List<Boolean> inPosition = new java.util.ArrayList<>();
        List<Double> leftCurrents = new java.util.ArrayList<>();
        List<Double> rightCurrents = new java.util.ArrayList<>();

        public void collectData(boolean inPosition, StatusSignal<Current> statorCurrent, StatusSignal<Current> current) {
            leftCurrents.add(statorCurrent.getValueAsDouble());
            rightCurrents.add(statorCurrent.getValueAsDouble());
        }

        @Override
        public Pair<Integer, Integer> getScore() {
            // FIXME: Is this where we want to have this happen
            List<Double> leftScore = AMDStats.detectOutliersZScore(leftCurrents, 70);
            List<Double> rightScore = AMDStats.detectOutliersZScore(rightCurrents, 70);
            int leftCount = leftScore.isEmpty() ? 0 : 1;
            int rightCount = rightScore.isEmpty() ? 0 : 2;
            return new Pair<>(leftCount, rightCount);
        }
    }
}
