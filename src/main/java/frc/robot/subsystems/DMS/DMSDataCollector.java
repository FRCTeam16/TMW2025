package frc.robot.subsystems.DMS;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Collects data from the DMS for analysis.
 * <p>
 * Uses https://www.itl.nist.gov/div898/handbook/eda/section3/eda35h.htm
 */
public class DMSDataCollector {
    private final DriveInfo<List<Double>> currentData = new DriveInfo<>(new ArrayList<>());
    private final DriveInfo<List<Double>> velocityData = new DriveInfo<>(new ArrayList<>());

    static double[] calculateModifiedZ(List<Double> data) {
        Collections.sort(data);
        // assume even number of observations-
        double median = (data.get(1) + data.get(2)) / 2;
        double[] deviations = data.stream().mapToDouble(c -> c - median).toArray();
        double medianDeviation = (deviations[1] + deviations[2]) / 2;

        double[] modifiedZ = data.stream().mapToDouble(c -> (0.6745 * (c - median)) / medianDeviation).toArray();
        return modifiedZ;
    }

    public void addCurrent(double[] current) {
        currentData.FL.add(current[0]);
        currentData.FR.add(current[1]);
        currentData.RL.add(current[2]);
        currentData.RR.add(current[3]);
    }

    public void addVelocity(double[] velocity) {
        velocityData.FL.add(velocity[0]);
        velocityData.FR.add(velocity[1]);
        velocityData.RL.add(velocity[2]);
        velocityData.RR.add(velocity[3]);
    }
}
