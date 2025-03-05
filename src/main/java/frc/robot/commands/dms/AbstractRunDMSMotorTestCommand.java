package frc.robot.commands.dms;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.DMS.SwerveDataCollector;
import frc.robot.util.BSLogger;

import static edu.wpi.first.units.Units.Seconds;

public abstract class AbstractRunDMSMotorTestCommand extends Command {

    private final Time runTime = Seconds.of(3.0);
    private final Time spinUpTime = Seconds.of(0.5);
    private final Timer timer = new Timer();
    private final SwerveDataCollector swerveDataCollector;


    public AbstractRunDMSMotorTestCommand(SwerveDataCollector swerveDataCollector) {
        addRequirements(Subsystems.swerveSubsystem);
        this.swerveDataCollector = swerveDataCollector;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        BSLogger.log("AbstractRunDMSMotorTestCommand", "initialize");
        timer.start();
//        startMotors();
    }

    /**
     * Start the motors for the test. TODO: Refactor to indicate it is run every period
     */
    abstract void startMotors();

    abstract void stopMotors();

    abstract double[] getMotorCurrents();

    abstract double[] getMotorVelocities();

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        BSLogger.log("AbstractRunDMSMotorTestCommand", "execute");
        startMotors();
        if (!timer.hasElapsed(spinUpTime.in(Seconds))) {
            BSLogger.log("AbstractRunDMSMotorTestCommand", "waiting....");
            return;
        }

        double[] current = getMotorCurrents();
        double[] velocity = getMotorVelocities();
        this.swerveDataCollector.addCurrent(current);
        this.swerveDataCollector.addVelocity(velocity);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        BSLogger.log("AbstractRunDMSMotorTestCommand", "end");
        stopMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(runTime.in(Seconds));
    }
}
