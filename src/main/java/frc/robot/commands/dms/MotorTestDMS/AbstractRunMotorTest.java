package frc.robot.commands.dms.MotorTestDMS;

import frc.robot.subsystems.DMS.AbstractDataCollector;

public abstract class AbstractRunMotorTest<T extends AbstractDataCollector<?>> {
    protected final T dataCollector;
    public AbstractRunMotorTest(T dataCollector) {
        this.dataCollector = dataCollector;
    }
    protected abstract void startMotor();
    protected abstract void stopMotor();
    protected abstract double[] getMotorCurrents();
}
