package frc.robot.subsystems;

import frc.robot.subsystems.amd.AbstractDataCollector;

public interface AMD<T extends AbstractDataCollector<?>> {
    void collectAMDData(T dataCollector);
}
