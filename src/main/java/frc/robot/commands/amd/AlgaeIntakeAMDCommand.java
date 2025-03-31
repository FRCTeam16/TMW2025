package frc.robot.commands.amd;

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

public class AlgaeIntakeAMDCommand extends Command {
    private final Time runTime = Seconds.of(3.0);
    private final Time spinUpTime = Seconds.of(0.5);
    private final Timer timer = new Timer();
    private final AlgaeIntakeDataCollector dataCollector = new AlgaeIntakeDataCollector();

    public AlgaeIntakeAMDCommand() {
        addRequirements(Subsystems.algaeIntake);
    }


    @Override
    public void initialize() {
        BSLogger.log("AlgaeIntakeAMDCommand", "initialize");
        Subsystems.ledSubsystem.getAMDSerialData().startAMDPhase(AMDSerialData.AMDPhase.AlgaeIntake);
        timer.restart();
    }

    @Override
    public void execute() {
        super.execute();
        Subsystems.algaeIntake.runAMD();
        if (!timer.hasElapsed(spinUpTime.in(Seconds))) {
            BSLogger.log("AlgaeIntakeAMDCommand", "spin up");
            return;
        }
        Subsystems.algaeIntake.collectAMDData(dataCollector);
        dataCollector.report();
    }

    @Override
    public void end(boolean interrupted) {
        Subsystems.ledSubsystem.getAMDSerialData().startAMDPhase(AMDSerialData.AMDPhase.AMDEnd);
        dataCollector.report();
        Subsystems.algaeIntake.stopAlgae();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(runTime.in(Seconds));
    }


    /**
     * Collects data from the Algae Intake for analysis.
     */
    public static class AlgaeIntakeDataCollector extends AbstractDataCollector<Integer> {
        List<Double> currents = new ArrayList<>();

        LinearFilter filter = LinearFilter.movingAverage(5);


        @Override
        public Integer getScore() {
            // 1 green < 15
            // 2 yellow < 25
            // 3 red > 25
            // 4 red

            double topAvg = Math.abs(filter.lastValue());
            int topScore;
            if (topAvg > 25) {
                topScore = 3;
            } else if (topAvg > 15) {
                topScore = 2;
            } else {
                topScore = 1;
            }


            return topScore;
        }

        public void addCurrents(double amps) {
            currents.add(amps);
            filter.calculate(amps);
        }

        public void report() {
            int score = this.getScore();
            SmartDashboard.putNumber("AMD/AlgaeIntakeAMDScore", score);
            Subsystems.ledSubsystem.getAMDSerialData().submitAlgaeIntakeScore(score);
        }
    }
}
