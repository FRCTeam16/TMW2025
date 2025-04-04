package frc.robot.commands.amd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.subsystems.Intake.AlgaeArm;
import frc.robot.subsystems.Intake.AlgaeArm.AlgaeArmPosition;
import frc.robot.subsystems.amd.AMDSerialData;
import frc.robot.subsystems.amd.AMDStats;
import frc.robot.subsystems.amd.AbstractDataCollector;
import frc.robot.util.BSLogger;

import java.util.List;

public class AlgaeArmAMDCommand extends Command {
    private final AlgaeArmDataCollector dataCollector = new AlgaeArmDataCollector();
    private Command amdCommand;

    public AlgaeArmAMDCommand() {
        // No subsystem requirements needed - the internal commands will handle that
    }

    @Override
    public void initialize() {
        amdCommand = createAMDCommand().withTimeout(5.0);
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

    /**
     * Creates the complete AMD command sequence
     */
    private Command createAMDCommand() {
        // Base command that runs in parallel
        Command baseCommand = Commands.sequence(
                Commands.runOnce(() ->
                        Subsystems.ledSubsystem.getAMDSerialData().startAMDPhase(AMDSerialData.AMDPhase.AlgaeIntake)
                ),
                Commands.deadline(
                                new WaitCommand(3.0),
                                new AlgaeArm.SetArmPositionCommand(AlgaeArm.AlgaeArmPosition.Ground),
                                collectDataCommand().repeatedly()
                        )
                        .until(() -> Subsystems.algaeArm.isInPosition())
                        .handleInterrupt(() -> dataCollector.setTimedOut(true)),
                new AlgaeArm.SetArmPositionCommand(AlgaeArmPosition.Up)
        );
        return baseCommand;
    }

    /**
     * Creates a command to collect data once
     */
    private Command collectDataCommand() {
        return Commands.runOnce(() -> {
            Subsystems.algaeArm.collectAMDData(dataCollector);
            dataCollector.report();
        });
    }

    public static class AlgaeArmDataCollector extends AbstractDataCollector<Integer> {
        private final List<Double> motorCurrents = new java.util.ArrayList<>();
        private boolean timedOut;

        public void collectData(double valueAsDouble) {
            BSLogger.log("AlgaeArmAMDCommand", "Motor current: " + valueAsDouble);
            motorCurrents.add(valueAsDouble);
        }

        @Override
        public Integer getScore() {
            if (timedOut) {
                return 2;
            }
            List<Double> outlierValues = AMDStats.detectOutliersMAD(motorCurrents, 3.5);
            if (outlierValues.size() > 10) {
                return 2;
            }
            return 1;
        }

        public void report() {
            int score = getScore();
            Subsystems.ledSubsystem.getAMDSerialData().submitAlgaeArmScore(score);
        }

        public void setTimedOut(boolean timedOut) {
            this.timedOut = true;
        }
    }
}