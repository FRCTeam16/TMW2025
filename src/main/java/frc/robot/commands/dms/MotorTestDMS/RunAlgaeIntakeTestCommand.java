package frc.robot.commands.dms.MotorTestDMS;

import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import frc.robot.Subsystems;
import frc.robot.subsystems.DMS.SingleMotorDataCollector;

class RunAlgaeIntakeTestCommand extends AbstractRunMotorTest<SingleMotorDataCollector> {

    public RunAlgaeIntakeTestCommand(SingleMotorDataCollector dmsDataCollector) {
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
        // FIXME
        return new double[0];
//            return new double[]{((CoreTalonFX) Subsystems.algaeIntake.getMotor()).getStatorCurrent().getValueAsDouble()};
        }

  
}
