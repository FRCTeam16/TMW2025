package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.util.BSLogger;

public class IntakeCoralCommand extends Command {
    public static final String STOP_CORAL_INTAKE_TASK = "StopCoralIntakeTask";
    private final CoralIntake coralIntake;
    int step = 1;

    public IntakeCoralCommand() {
        this.coralIntake = Subsystems.coralIntake;
        addRequirements(coralIntake);
    }

    @Override
    public void initialize() {
        BSLogger.log("IntakeCoralCommand", "**** STARTING ****");
        step = 1;

        if (Subsystems.coralIntake.coralDetectedAtBottomSensor()) {
            BSLogger.log("IntakeCoralCommand", "Coral already detected");
            return;
        }

        // Start with fast intake
        coralIntake.intakeFast();
        BSLogger.log("IntakeCoralCommand", "**** EXECUTING CORAL INTAKE STEP: " + step);
    }

    @Override
    public void execute() {
        BSLogger.log("IntakeCoralCommand", "**** EXECUTING CORAL INTAKE STEP: " + step);

        if (step == 1) {
            //if first laser sees coral while default action: change action to action 2
            if (coralIntake.coralDetectedAtTopSensor()) {
                BSLogger.log("IntakeCoralCommandAsync", "Async detect & stop started");
                Subsystems.asyncManager.register(STOP_CORAL_INTAKE_TASK, () -> {
                    if (Subsystems.coralIntake.coralDetectedAtBottomSensor()) {
                        BSLogger.log("IntakeCoralCommandAsync", "Async STOPPING INTAKE CMD");
                        step = 3;
                        Subsystems.coralIntake.stop();
                    }
                });
                BSLogger.log("IntakeCoralCommand", "Coral detected at first sensor, slowing intake");
                coralIntake.intakeSlow();
                step = 2;
                BSLogger.log("IntakeCoralCommand", "**** EXECUTING CORAL INTAKE STEP: " + step);

            }
        }

        //second action when intake: runForward(slow)
        if (step == 2) {
            //if second laser sees coral while second action: change action to action 3
            if (coralIntake.coralDetectedAtBottomSensor()) {
                BSLogger.log("IntakeCoralCommand", "Coral detected at second sensor, stopping intake");
                coralIntake.stop();
                step = 3;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return coralIntake.coralDetectedAtBottomSensor() || step == 3;
    }

    @Override
    public void end(boolean interrupted) {
        Subsystems.asyncManager.unregister(STOP_CORAL_INTAKE_TASK);
        coralIntake.stop();
    }
}
