package frc.robot.commands.dms.MotorTestDMS;

import frc.robot.Subsystems;
import frc.robot.subsystems.DMS.DMSDataCollector;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;

class RunCoralIntakeTestCommand extends AbstractRunMotorTest {
    
    private final MedianFilter currentFilter;
    
    public RunCoralIntakeTestCommand(DMSDataCollector dmsDataCollector) {
        super(dmsDataCollector);
        this.currentFilter = new MedianFilter(5); // 5-sample median filter
    }

    @Override
    protected void startMotor() {
        Subsystems.coralIntake.intakeCoralCommand();  // Changed from stopCommand() to start the motor
    }

    @Override
    protected void stopMotor() {
        Subsystems.coralIntake.stopCommand();
    }

    @Override
    protected double[] getMotorCurrents() {
        double rawCurrent = Subsystems.coralIntake.();
        double filteredCurrent = currentFilter.calculate(rawCurrent); // Apply median filter
        return new double[]{filteredCurrent}; // Return the filtered value in an array
    }
}
