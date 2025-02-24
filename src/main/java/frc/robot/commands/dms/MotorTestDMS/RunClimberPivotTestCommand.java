package frc.robot.commands.dms.MotorTestDMS;
import frc.robot.Subsystems;
import frc.robot.subsystems.DMS.DMSDataCollector;

class RunClimberPivotTestCommand extends AbstractRunMotorTest {

    public RunClimberPivotTestCommand(DMSDataCollector dmsDataCollector) {
        super(dmsDataCollector);
    }

    @Override
        protected
        void startMotor() {
            Subsystems.climber.openLoopUpDefault(); 
        }

    @Override
        protected
        void stopMotor() {
            Subsystems.climber.openLoopDownDefault();
        }

    @Override
        protected
        double[] getMotorCurrents() {
           // return new double[]{((TalonFX) Subsystems.algaeArm.getMotorPosition()).getStatorCurrent().getValueAsDouble()};
           return null;
        }

  
}