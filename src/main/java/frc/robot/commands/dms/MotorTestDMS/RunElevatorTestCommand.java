package frc.robot.commands.dms.MotorTestDMS;
import frc.robot.Subsystems;
import frc.robot.subsystems.DMS.DMSDataCollector;

class RunElevatorTestCommand extends AbstractRunMotorTest {

    public RunElevatorTestCommand(DMSDataCollector dmsDataCollector) {
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
          //  return new double[]{(() Subsystems.algaeArm.getMotorPosition()).getStatorCurrent().getValueAsDouble()};
        return null;
        }

  
}
