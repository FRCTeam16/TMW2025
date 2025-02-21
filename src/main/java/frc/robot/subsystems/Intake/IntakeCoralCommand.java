package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.util.BSLogger;

public class IntakeCoralCommand extends Command {
    private final CoralIntake coralIntake;
    int step = 1;

    public IntakeCoralCommand() {
        this.coralIntake = Subsystems.coralIntake;
        addRequirements(coralIntake, Subsystems.funnelSubsystem);
    }

    @Override
    public void initialize() {
        BSLogger.log("CoralIntakeCommand", "**** STARTING ****");
        step = 1;
        // Start with fast intake
        coralIntake.intakeFast();
        Subsystems.funnelSubsystem.startConveyor();
    }

    @Override
    public void execute() {
        BSLogger.log("CoralIntakeCommand", "**** EXECUTING CORAL INTAKE STEP: " + step);
        //default action when intake: runForward(fast)
        if (step == 1) {
            //if first laser sees coral while default action: change action to action 2
            if (coralIntake.coralDetectedAtFirstLaser()) {
                BSLogger.log("CoralIntakeCommand", "Coral detected at first laser");
                coralIntake.intakeSlow();
                step = 2;
            }
        }

        //second action when intake: runForward(slow)
        if (step == 2) {
            //if second laser sees coral while second action: change action to action 3
            if (coralIntake.coralDetectedAtSecondLaser()) {
                BSLogger.log("CoralIntakeCommand", "Coral detected at second laser");
                coralIntake.stop();
                step = 3;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return coralIntake.coralDetectedAtSecondLaser() || step == 3;
    }

    @Override
    public void end(boolean interrupted) {
        coralIntake.stop();
        Subsystems.funnelSubsystem.stopConveyor();
    }
}
