package frc.robot.commands.dms.MotorTestDMS;
import frc.robot.Subsystems;
import frc.robot.subsystems.DMS.DMSDataCollector;

class RunCoralIntakeTestCommand extends AbstractRunMotorTest {

    public RunCoralIntakeTestCommand(DMSDataCollector dmsDataCollector) {
        super(dmsDataCollector);
    }

    @Override
        protected
        void startMotor() {
            Subsystems.coralIntake.stopCommand(); 
        }

    @Override
        protected
        void stopMotor() {
            Subsystems.coralIntake.ejectCommand();
        }

    @Override
        protected
        double[] getMotorCurrents() {
           // return new double[]{((TalonFX) Subsystems.algaeArm.getMotorPosition()).getStatorCurrent().getValueAsDouble()};
           return null;
        }

  
}