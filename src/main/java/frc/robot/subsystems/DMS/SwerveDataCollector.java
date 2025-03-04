package frc.robot.subsystems.DMS;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.DoubleToIntFunction;

/**
 * Collects data from the DMS for analysis.
 * <p>
 * Uses https://www.itl.nist.gov/div898/handbook/eda/section3/eda35h.htm
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

        DriveInfo<Boolean> currentOutliers = detectOutliers(currentFLArray, currentFRArray, currentRLArray, currentRRArray);
        DriveInfo<Boolean> velocityOutliers = detectOutliers(velocityFLArray, velocityFRArray, velocityRLArray, velocityRRArray);

        DriveInfo<Integer> score = new DriveInfo<>(0);
        score.FL = calculateStatus(currentOutliers.FL, velocityOutliers.FL);
        score.FR = calculateStatus(currentOutliers.FR, velocityOutliers.FR);
        score.RL = calculateStatus(currentOutliers.RL, velocityOutliers.RL);
        score.RR = calculateStatus(currentOutliers.RR, velocityOutliers.RR);

        return score;
    }

    /**
     * Detects which motor(s) are outliers using the MAD approach.
     * Returns a list of motor labels that exceed the chosen z-score threshold.
     */
    public static DriveInfo<Boolean> detectOutliers(double[] FL, double[] FR, double[] RL, double[] RR) {
        // 1. Concatenate all motor arrays into one
        double[] allData = concatArrays(FL, FR, RL, RR);

        // 2. Compute global median
        double globalMedian = median(allData);

        // 3. Compute absolute deviations from the global median
        double[] absDevs = new double[allData.length];
        for (int i = 0; i < allData.length; i++) {
            absDevs[i] = Math.abs(allData[i] - globalMedian);
        }

        // 4. Compute MAD (median of absolute deviations)
        double MAD = median(absDevs);

        // 5. Compute each motor's median
        double medianFL = median(FL);
        double medianFR = median(FR);
        double medianRL = median(RL);
        double medianRR = median(RR);

        // 6. Compute z-score for each motor's median
        // Use modified Z
        double zFL = 0.6745 * (Math.abs(medianFL - globalMedian)) / MAD;
        double zFR = 0.6745 * (Math.abs(medianFR - globalMedian)) / MAD;
        double zRL = 0.6745 * (Math.abs(medianRL - globalMedian)) / MAD;
        double zRR = 0.6745 * (Math.abs(medianRR - globalMedian)) / MAD;

        // 7. Compare against a threshold (e.g., 3.0)
        double threshold = 3.0;

        DriveInfo<Boolean> outliers = new DriveInfo<>(false);
        if (zFL > threshold) outliers.FL = true;
        if (zFR > threshold) outliers.FR = true;
        if (zRL > threshold) outliers.RL = true;
        if (zRR > threshold) outliers.RR = true;

        return outliers;
    }

    /**
     * Computes the median of a double array.
     * If the array has an even length, returns the average of the two middle values.
     */
    public static double median(double[] data) {
        if (data == null || data.length == 0) {
            throw new IllegalArgumentException("Data array is empty or null");
        }

        double[] copy = Arrays.copyOf(data, data.length);
        Arrays.sort(copy);

        int n = copy.length;
        if (n % 2 == 1) {
            // Odd length
            return copy[n / 2];
        } else {
            // Even length: average of the two middle values
            double lower = copy[(n / 2) - 1];
            double upper = copy[n / 2];
            return (lower + upper) / 2.0;
        }
    }

    /**
     * Concatenates multiple double arrays into one.
     */
    public static double[] concatArrays(double[]... arrays) {
        int totalLength = 0;
        for (double[] arr : arrays) {
            totalLength += arr.length;
        }

        double[] result = new double[totalLength];
        int offset = 0;
        for (double[] arr : arrays) {
            System.arraycopy(arr, 0, result, offset, arr.length);
            offset += arr.length;
        }
        return result;
    }

    private double[] calculateModifiedZ(List<Double> data) {
        Collections.sort(data);
        // assume even number of observations
        double median = (data.get(1) + data.get(2)) / 2;
        double[] deviations = data.stream().mapToDouble(c -> c - median).toArray();
        double medianDeviation = (deviations[1] + deviations[2]) / 2;

        double[] modifiedZ = data.stream().mapToDouble(c -> (0.6745 * (c - median) ) / medianDeviation).toArray();
        return modifiedZ;
    }

    /*
     * 0: No outliers in current or velocity
     * 1: Outliers in velocity only
     * 2: Outliers in current only
     * 3: Outliers in both current and velocity
     * 4: Error
     */
    private int calculateStatus(Boolean currentStatus, Boolean velocityStatus) {
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
