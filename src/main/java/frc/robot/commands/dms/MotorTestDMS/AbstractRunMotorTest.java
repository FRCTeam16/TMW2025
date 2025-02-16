package frc.robot.commands.dms.MotorTestDMS;

import frc.robot.subsystems.DMS.DMSDataCollector;

public abstract class AbstractRunMotorTest {
    public AbstractRunMotorTest(DMSDataCollector dmsDataCollector) {
        //TODO Auto-generated constructor stub
    }
    protected abstract void startMotor();
    protected abstract void stopMotor();
    protected abstract double[] getMotorCurrents();
}
