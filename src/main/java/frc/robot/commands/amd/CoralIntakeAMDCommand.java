package frc.robot.commands.amd;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.amd.AMDSerialData;
import frc.robot.subsystems.amd.AbstractDataCollector;
import frc.robot.util.BSLogger;

import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.units.Units.Seconds;

public class CoralIntakeAMDCommand extends Command {
    private final Time runTime = Seconds.of(3.0);
    private final Time spinUpTime = Seconds.of(0.5);
    private final Timer timer = new Timer();
    private final CoralIntakeDataCollector dataCollector = new CoralIntakeDataCollector();

    public CoralIntakeAMDCommand() {
        addRequirements(Subsystems.coralIntake);
    }

    public CoralIntakeDataCollector getDataCollector() {
        return dataCollector;
    }

    @Override
    public void initialize() {
        BSLogger.log("CoralIntakeAMDCommand", "initialize");
        Subsystems.ledSubsystem.getAMDSerialData().startAMDPhase(AMDSerialData.AMDPhase.CoralShooter);
        timer.restart();
    }

    @Override
    public void execute() {
        super.execute();
        Subsystems.coralIntake.runAMD();
        if (!timer.hasElapsed(spinUpTime.in(Seconds))) {
            BSLogger.log("CoralIntakeAMDCommand", "spin up");
            return;
        }
        Subsystems.coralIntake.collectAMDData(dataCollector);
        dataCollector.report();
    }

    @Override
    public void end(boolean interrupted) {
        Subsystems.ledSubsystem.getAMDSerialData().startAMDPhase(AMDSerialData.AMDPhase.AMDEnd);
        dataCollector.report();
        Subsystems.coralIntake.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(runTime.in(Seconds));
    }


    /**
     * Collects data from the Coral Intake for analysis.
     */
    public static class CoralIntakeDataCollector extends AbstractDataCollector<Pair<Integer, Integer>> {
        List<Double> topCurrents = new ArrayList<>();
        List<Double> bottomCurrents = new ArrayList<>();

        LinearFilter topFilter = LinearFilter.movingAverage(5);
        LinearFilter bottomFilter = LinearFilter.movingAverage(5);


        @Override
        public Pair<Integer, Integer> getScore() {
            // 1 green < 15
            // 2 yellow < 25
            // 3 red > 25
            // 4 red

            double topAvg = Math.abs(topFilter.lastValue());
            int topScore;
            if (topAvg > 25) {
                topScore = 3;
            } else if (topAvg > 15) {
                topScore = 2;
            } else {
                topScore = 1;
            }
            int bottomScore;
            if (topAvg > 25) {
                bottomScore = 3;
            } else if (topAvg > 15) {
                bottomScore = 2;
            } else {
                bottomScore = 1;
            }

            return Pair.of(topScore, bottomScore);
        }

        public void addCurrents(double topAmps, double bottomAmps) {
            topCurrents.add(topAmps);
            bottomCurrents.add(bottomAmps);

            topFilter.calculate(topAmps);
            bottomFilter.calculate(bottomAmps);
        }

        public void report() {
            Pair<Integer, Integer> score = this.getScore();
            SmartDashboard.putNumber("AMD/CoralIntakeAMDScore", score.getFirst());

            Subsystems.ledSubsystem.getAMDSerialData().submitLeftCoralScore(score.getFirst());
            Subsystems.ledSubsystem.getAMDSerialData().submitRightCoralScore(score.getSecond());
        }
    }
}
