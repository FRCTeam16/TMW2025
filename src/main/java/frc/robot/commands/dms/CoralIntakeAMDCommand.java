package frc.robot.commands.dms;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.DMS.AbstractDataCollector;
import frc.robot.subsystems.DMS.LEDSubsystem;
import frc.robot.subsystems.DMS.SwerveDataCollector;
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
        Subsystems.ledSubsystem.startAMDPhase(LEDSubsystem.AMDPhase.CoralShooter);
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
        Subsystems.coralIntake.collectAMD(dataCollector);
    }

    @Override
    public void end(boolean interrupted) {
        int score = dataCollector.getScore();
        SmartDashboard.putNumber("AMD/CoralIntakeAMDScore", score);
        Subsystems.ledSubsystem.submitCoralAMDScore(score);
        Subsystems.ledSubsystem.startAMDPhase(LEDSubsystem.AMDPhase.Comm);
        Subsystems.coralIntake.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(runTime.in(Seconds));
    }


    public static class CoralIntakeDataCollector extends AbstractDataCollector<Integer> {
        List<Double> topCurrents = new ArrayList<>();
        List<Double> bottomCurrents = new ArrayList<>();

        LinearFilter topFilter = LinearFilter.movingAverage(5);
        LinearFilter bottomFilter = LinearFilter.movingAverage(5);


        @Override
        public Integer getScore() {
            boolean topOutlier = Math.abs(topFilter.lastValue()) > 20;
            boolean bottomOutlier = Math.abs(bottomFilter.lastValue()) > 20;
            int score = (topOutlier ? 1 : 0) + (bottomOutlier ? 2 : 0);
            return score;
        }

        public void addCurrents(double topAmps, double bottomAmps) {
            topCurrents.add(topAmps);
            bottomCurrents.add(bottomAmps);

            topFilter.calculate(topAmps);
            bottomFilter.calculate(bottomAmps);
        }
    }
}
