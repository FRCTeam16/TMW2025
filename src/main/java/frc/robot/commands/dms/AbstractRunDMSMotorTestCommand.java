package frc.robot.commands.dms;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.DMS.DMSDataCollector;

import static edu.wpi.first.units.Units.Seconds;

public abstract class AbstractRunDMSMotorTestCommand extends Command {

    private final Time runTime = Seconds.of(3.0);
    private final Time spinUpTime = Seconds.of(0.5);
    private final Timer timer = new Timer();
    private final DMSDataCollector dmsDataCollector;


    public AbstractRunDMSMotorTestCommand(DMSDataCollector dmsDataCollector) {
        addRequirements(Subsystems.swerveSubsystem);
        this.dmsDataCollector = dmsDataCollector;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.start();
       startMotors();

    }

    abstract void startMotors();
    abstract void stopMotors();
    abstract double[] getMotorCurrents();
    abstract double[] getMotorVelocities();

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!timer.hasElapsed(spinUpTime.in(Seconds))) {
            return;
        }
        double[] current = getMotorCurrents();
        double[] velocity = getMotorVelocities();
        this.dmsDataCollector.addCurrent(current);
        this.dmsDataCollector.addVelocity(velocity);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
       stopMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(runTime.in(Seconds));
    }
}
