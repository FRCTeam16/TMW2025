package frc.robot.commands.dms.MotorTestDMS;
import frc.robot.Subsystems;
import frc.robot.subsystems.DMS.DMSDataCollector;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;

class RunAlgaeIntakeTestCommand extends AbstractRunMotorTest {

    public RunAlgaeIntakeTestCommand(DMSDataCollector dmsDataCollector) {
        super(dmsDataCollector);
    }

    @Override
        protected
        void startMotor() {
            Subsystems.algaeIntake.intakeCommand(); 
        }

    @Override
        protected
        void stopMotor() {
            Subsystems.algaeIntake.stopCommand();
        }

    @Override
        protected
        double[] getMotorCurrents() {
            return new double[]{((CoreTalonFX) Subsystems.algaeIntake.getMotor()).getStatorCurrent().getValueAsDouble()};
        }

  
}
