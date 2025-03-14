package frc.robot.subsystems.DMS;

import frc.robot.util.BSLogger;

import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.DoubleToIntFunction;

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

        DriveInfo<Boolean> currentOutliers = detectOutliers(currentFLArray, currentFRArray, currentRLArray, currentRRArray, "current");
        DriveInfo<Boolean> velocityOutliers = detectOutliers(velocityFLArray, velocityFRArray, velocityRLArray, velocityRRArray, "velocity");



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
    public static DriveInfo<Boolean> detectOutliers(double[] FL, double[] FR, double[] RL, double[] RR, String label) {
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

        // Handle zero or near-zero MAD
        double minMAD = 1e-6; // Minimum threshold for MAD
        if (MAD < minMAD) {
            MAD = minMAD;
        }

        // 5. Compute each motor's median
        double medianFL = median(FL);
        double medianFR = median(FR);
        double medianRL = median(RL);
        double medianRR = median(RR);

        try(FileWriter out = new FileWriter("/home/lvuser/" + label + ".csv")) {
            out.append("FL,FR,RL,RR\n");
            for (int i=0;i<FL.length;i++) {
                out.append(Double.toString(FL[i])).append(",")
                        .append(Double.toString(FR[i])).append(",")
                        .append(Double.toString(RL[i])).append(",")
                        .append(Double.toString(RR[i])).append("\n");
            }
        } catch (Exception e) {
            BSLogger.log("SwerveDataCollector", "Error writing CSV File");
        }

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

        try(FileWriter out = new FileWriter("/home/lvuser/medians"+label+".csv")) {
            out.append("MAD,Global,FL,FR,RL,RR,zFL,zFR,zRL,zRR,oFL,oFR,oRL,oRR\n")
                    .append(Double.toString(MAD)).append(',')
                    .append(Double.toString(globalMedian)).append(',')
                    .append(Double.toString(medianFL)).append(',')
                    .append(Double.toString(medianFR)).append(',')
                    .append(Double.toString(medianRL)).append(',')
                    .append(Double.toString(medianRR)).append(',')
                    .append(Double.toString(zFL)).append(',')
                    .append(Double.toString(zFR)).append(',')
                    .append(Double.toString(zRL)).append(',')
                    .append(Double.toString(zRR)).append(',')
                    .append(Boolean.toString(outliers.FL)).append(',')
                    .append(Boolean.toString(outliers.FR)).append(',')
                    .append(Boolean.toString(outliers.RL)).append(',')
                    .append(Boolean.toString(outliers.RR)).append('\n');
        } catch (Exception e) {
            BSLogger.log("SwerveDataCollector", "Error writing medians CSV File");
        }

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
