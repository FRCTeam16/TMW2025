package frc.robot.commands.dms.MotorTestDMS;
import frc.robot.Subsystems;
import frc.robot.subsystems.DMS.DMSDataCollector;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;

class RunAlgaeArmTestCommand extends AbstractRunMotorTest {

    public RunAlgaeArmTestCommand(DMSDataCollector dmsDataCollector) {
        super(dmsDataCollector);
    }

    @Override
        protected
        void startMotor() {
            Subsystems.algaeArm.openLoopUpCommand(); 
        }

    @Override
        protected
        void stopMotor() {
            Subsystems.algaeArm.openLoopUpCommand();
        }

    @Override
        protected
        double[] getMotorCurrents() {
            return new double[]{((TalonFX) Subsystems.algaeArm.getMotorPosition()).getStatorCurrent().getValueAsDouble()};
        }

  
}
