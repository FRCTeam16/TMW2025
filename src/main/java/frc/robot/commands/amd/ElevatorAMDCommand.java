package frc.robot.commands.amd;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.amd.AMDSerialData;
import frc.robot.subsystems.amd.AMDStats;
import frc.robot.subsystems.amd.AbstractDataCollector;
import frc.robot.util.BSLogger;

import java.util.List;


/**
 * TODO: run to l2/l3, pause, then home with timeout
 * send
 */
public class ElevatorAMDCommand extends Command {
    private ElevatorDataCollector dataCollector = new ElevatorDataCollector();
    private Timer timer = new Timer();
    private Command moveElevatorCommand;
    int phaseNum = 0;

    public ElevatorAMDCommand() {
        addRequirements(Subsystems.elevator);
    }

    @Override
    public void initialize() {
        Subsystems.ledSubsystem.getAMDSerialData().startAMDPhase(AMDSerialData.AMDPhase.Elevator);
        timer.stop();
        timer.reset();
        moveElevatorCommand = null;
        phaseNum = 1;
    }

    @Override
    public void execute() {
        if (phaseNum == 1) {
            BSLogger.log("ElevatorAMDCommand","phase1");
            if (moveElevatorCommand == null) {
                moveElevatorCommand = new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L2);
                moveElevatorCommand.schedule();
            } else if (!moveElevatorCommand.isScheduled() || moveElevatorCommand.isFinished()) {
                BSLogger.log("ElevatorAMDCommand", "phase1 finished");
                moveElevatorCommand.cancel();
                moveElevatorCommand = null;
                phaseNum++;
            }
        } else if (phaseNum == 2) {
            if (moveElevatorCommand == null) {
                moveElevatorCommand = new WaitCommand(4.0);
                moveElevatorCommand.schedule();
            } else if (!moveElevatorCommand.isScheduled() || moveElevatorCommand.isFinished()) {
                moveElevatorCommand.cancel();
                moveElevatorCommand = null;
                timer.start();
                phaseNum++;
            }
        } else if (phaseNum == 3) {
            if (moveElevatorCommand == null) {
                moveElevatorCommand = new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.Zero);
                moveElevatorCommand.schedule();
            } else if (!moveElevatorCommand.isScheduled() || moveElevatorCommand.isFinished()) {
                moveElevatorCommand.cancel();
                moveElevatorCommand = null;
                phaseNum++;
            }
        }
        Subsystems.elevator.collectAMDData(dataCollector);
    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(5.0)) {
            dataCollector.setTimedOut(true);
            return true;
        }
        return phaseNum == 4;
    }

    @Override
    public void end(boolean interrupted) {
        if (moveElevatorCommand != null) {
            moveElevatorCommand.cancel();
        }

        Pair<Integer, Integer> score = dataCollector.getScore();
        Subsystems.ledSubsystem.getAMDSerialData().submitElevatorScore(score.getFirst(), score.getSecond());
        Subsystems.ledSubsystem.getAMDSerialData().startAMDPhase(AMDSerialData.AMDPhase.AMDEnd);
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
            // FIXME: Is this where we want to have this happen
            List<Double> leftScore = AMDStats.detectOutliersZScore(leftCurrents, 70);
            List<Double> rightScore = AMDStats.detectOutliersZScore(rightCurrents, 70);
            int leftCount = leftScore.isEmpty() ? 0 : 1;
            int rightCount = rightScore.isEmpty() ? 0 : 2;
            return new Pair<>(leftCount, rightCount);
        }

        public void setTimedOut(boolean b) {
            this.timedOut = true;
        }
    }
}
