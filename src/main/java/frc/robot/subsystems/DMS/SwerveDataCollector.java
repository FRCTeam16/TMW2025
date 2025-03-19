package frc.robot.subsystems.DMS;

import java.util.ArrayList;
import java.util.List;

/**
 * Collects data from the DMS for analysis.
 * <p>
 * Uses https://www.itl.nist.gov/div898/handbook/eda/section3/eda35h.htm
 *
 * https://crispinagar.github.io/blogs/mad-anomaly-detection.html
 */
public class SwerveDataCollector extends AbstractDataCollector<DriveInfo<Integer>> {
    private final DriveInfo<List<Double>> currentData = new DriveInfo<>(new ArrayList<>());
    private final DriveInfo<List<Double>> velocityData = new DriveInfo<>(new ArrayList<>());


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

    @Override
    public DriveInfo<Integer> getScore() {
        double[] currentFLArray = currentData.FL.stream().mapToDouble(Double::doubleValue).toArray();
        double[] currentFRArray = currentData.FR.stream().mapToDouble(Double::doubleValue).toArray();
        double[] currentRLArray = currentData.RL.stream().mapToDouble(Double::doubleValue).toArray();
        double[] currentRRArray = currentData.RR.stream().mapToDouble(Double::doubleValue).toArray();

        double[] velocityFLArray = velocityData.FL.stream().mapToDouble(Double::doubleValue).toArray();
        double[] velocityFRArray = velocityData.FR.stream().mapToDouble(Double::doubleValue).toArray();
        double[] velocityRLArray = velocityData.RL.stream().mapToDouble(Double::doubleValue).toArray();
        double[] velocityRRArray = velocityData.RR.stream().mapToDouble(Double::doubleValue).toArray();

        DriveInfo<Boolean> currentOutliers = AMDStats.detectOutliers(currentFLArray, currentFRArray, currentRLArray, currentRRArray, "current");
        DriveInfo<Boolean> velocityOutliers = AMDStats.detectOutliers(velocityFLArray, velocityFRArray, velocityRLArray, velocityRRArray, "velocity");



        DriveInfo<Integer> score = new DriveInfo<>(0);
        score.FL = calculateStatus(currentOutliers.FL, velocityOutliers.FL);
        score.FR = calculateStatus(currentOutliers.FR, velocityOutliers.FR);
        score.RL = calculateStatus(currentOutliers.RL, velocityOutliers.RL);
        score.RR = calculateStatus(currentOutliers.RR, velocityOutliers.RR);

        return score;
    }

    /*
     * 0: No outliers in current or velocity
     * 1: Outliers in velocity only
     * 2: Outliers in current only
     * 3: Outliers in both current and velocity
     * 4: Error
     */
    public static int calculateStatus(Boolean currentStatus, Boolean velocityStatus) {
        return (currentStatus ? 2 : 0) + (velocityStatus ? 1 : 0);
//        if (!currentStatus && !velocityStatus) {
//            return 0;
//        } else if (!currentStatus && velocityStatus) {
//            return 1;
//        } else if (currentStatus && !velocityStatus) {
//            return 2;
//        } else if (currentStatus && velocityStatus) {
//            return 3;
//        } else {
//            return 4;
//        }
    }
}
