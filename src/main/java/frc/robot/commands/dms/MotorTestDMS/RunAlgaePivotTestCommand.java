package frc.robot.commands.dms.MotorTestDMS;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems;
import frc.robot.subsystems.DMS.SingleMotorDataCollector;
import frc.robot.subsystems.Intake.AlgaeArm;

class RunAlgaeArmTestCommand extends AbstractRunMotorTest<SingleMotorDataCollector> {

    public RunAlgaeArmTestCommand(SingleMotorDataCollector dmsDataCollector) {
        super(dmsDataCollector);
    }

    @Override
        protected
        void startMotor() {
//            Subsystems.algaeArm.openLoopUpCommand();
            var cmd = Subsystems.algaeArm.setArmPositionCommand(AlgaeArm.AlgaeArmPosition.Start);
            // maybe> CommandScheduler.getInstance().schedule(cmd);
        }

    @Override
        protected
        void stopMotor() {
//            Subsystems.algaeArm.openLoopDownCommand();
        }

    @Override
        protected
        double[] getMotorCurrents() {

           // return new double[]{((TalonFX) Subsystems.algaeArm.getMotorPosition()).getStatorCurrent().getValueAsDouble()};
           return null;
        }

  
}
