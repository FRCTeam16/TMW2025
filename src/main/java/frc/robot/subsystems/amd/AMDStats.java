package frc.robot.subsystems.amd;

import frc.robot.util.BSLogger;

import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Utility class for calculating AMD related statistics
 */
public class AMDStats {

    /**
     * Detects outliers using the Z-Score (3-Sigma) rule.
     * @param data The list of numeric values.
     * @param threshold Typically 3 for a 3-sigma rule.
     * @return A list of values that are considered outliers (|z| > threshold).
     */
    public static List<Double> detectOutliersZScore(List<Double> data, double threshold) {
        double mean = mean(data);
        double stdDev = standardDeviation(data, mean);

        List<Double> outliers = new ArrayList<>();
        for (double val : data) {
            double z = (val - mean) / stdDev;
            if (Math.abs(z) > threshold) {
                outliers.add(val);
            }
        }
        return outliers;
    }

    public static void removeKnownCSVFiles() {
        try {
            new File("/home/lvuser/current.csv").delete();
            new File("home/lvuser/velocity.csv").delete();
        } catch (Exception ex) {
            BSLogger.log("AMDStats", "Error removing file: " + ex.getMessage());
        }
    }


    /**
     * Detects which motor(s) are outliers using the MAD approach.
     * Returns a list of motor labels that exceed the chosen z-score threshold.
     */
    public static DriveInfo<Boolean> detectOutliers(double[] FL, double[] FR, double[] RL, double[] RR, String label) {
        if (FL.length == 0 || FR.length == 0 || RL.length == 0 || RR.length == 0) {
            BSLogger.log("AMDStats", "detectOutliers needs at least a data point in all arrays");
            return new DriveInfo<Boolean>(false);
        }

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

        // 5. Compute modified Z-Score for each data point
        double threshold = 3.0;
        DriveInfo<Boolean> outliers = new DriveInfo<>(false);

        for (int i = 0; i < FL.length; i++) {
            double zFL = 0.6745 * (FL[i] - globalMedian) / MAD;
            double zFR = 0.6745 * (FR[i] - globalMedian) / MAD;
            double zRL = 0.6745 * (RL[i] - globalMedian) / MAD;
            double zRR = 0.6745 * (RR[i] - globalMedian) / MAD;

            if (Math.abs(zFL) > threshold) outliers.FL = true;
            if (Math.abs(zFR) > threshold) outliers.FR = true;
            if (Math.abs(zRL) > threshold) outliers.RL = true;
            if (Math.abs(zRR) > threshold) outliers.RR = true;

//            if (Math.abs(zFL) > threshold) {
//                System.out.println("Outlier FL: " + FL[i] + " zFL: " + zFL);
//            }
//            if (Math.abs(zFR) > threshold) {
//                System.out.println("Outlier FR: " + FR[i] + " zFR: " + zFR);
//            }
//            if (Math.abs(zRL) > threshold) {
//                System.out.println("Outlier RL: " + RL[i] + " zRL: " + zRL);
//            }
//            if (Math.abs(zRR) > threshold) {
//                System.out.println("Outlier RR: " + RR[i] + " zRR: " + zRR);
//            }
        }

        try(FileWriter out = new FileWriter("/home/lvuser/" + label + ".csv", true)) {
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

//        try(FileWriter out = new FileWriter("/home/lvuser/medians"+label+".csv")) {
//            out.append("MAD,Global,FL,FR,RL,RR,zFL,zFR,zRL,zRR,oFL,oFR,oRL,oRR\n")
//                    .append(Double.toString(MAD)).append(',')
//                    .append(Double.toString(globalMedian)).append(',')
//                    .append(Double.toString(medianFL)).append(',')
//                    .append(Double.toString(medianFR)).append(',')
//                    .append(Double.toString(medianRL)).append(',')
//                    .append(Double.toString(medianRR)).append(',')
//                    .append(Double.toString(zFL)).append(',')
//                    .append(Double.toString(zFR)).append(',')
//                    .append(Double.toString(zRL)).append(',')
//                    .append(Double.toString(zRR)).append(',')
//                    .append(Boolean.toString(outliers.FL)).append(',')
//                    .append(Boolean.toString(outliers.FR)).append(',')
//                    .append(Boolean.toString(outliers.RL)).append(',')
//                    .append(Boolean.toString(outliers.RR)).append('\n');
//        } catch (Exception e) {
//            BSLogger.log("SwerveDataCollector", "Error writing medians CSV File");
//        }

        return outliers;
    }

    /**
     * Detects outliers using the MAD (Modified Z-Score) method.
     * @param data The list of numeric values.
     * @param threshold Often 3.5 is used as a cutoff for modified z-scores.
     * @return A list of values that are considered outliers (|M| > threshold).
     */
    public static List<Double> detectOutliersMAD(List<Double> data, double threshold) {
        // 1. Calculate median
        double median = median(data);

        // 2. Calculate absolute deviations from the median
        List<Double> absDeviations = new ArrayList<>();
        for (double val : data) {
            absDeviations.add(Math.abs(val - median));
        }

        // 3. Calculate MAD (median of the absolute deviations)
        double madValue = median(absDeviations);

        // Safeguard to avoid division by zero if MAD is 0
        if (madValue == 0) {
            madValue = 1e-9;
        }

        // 4. Calculate the modified z-score for each data point:
        //    M_i = 0.6745 * (x_i - median) / MAD
        List<Double> outliers = new ArrayList<>();
        for (double val : data) {
            double modifiedZ = 0.6745 * (val - median) / madValue;
            if (Math.abs(modifiedZ) > threshold) {
                outliers.add(val);
            }
        }
        return outliers;
    }


    /**
     * Computes the mean of a list of doubles.
     */
    public static double mean(List<Double> data) {
        if (data.isEmpty()) return 0.0;
        double sum = 0.0;
        for (double val : data) {
            sum += val;
        }
        return sum / data.size();
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
     * Computes the median of a list of doubles.
     */
    public static double median(List<Double> data) {
        if (data.isEmpty()) return 0.0;

        List<Double> sorted = new ArrayList<>(data);
        Collections.sort(sorted);
        int n = sorted.size();
        if (n % 2 == 1) {
            // Odd number of elements
            return sorted.get(n / 2);
        } else {
            // Even number of elements
            double lower = sorted.get((n / 2) - 1);
            double upper = sorted.get(n / 2);
            return (lower + upper) / 2.0;
        }
    }

    /**
     * Computes the sample standard deviation of a list of doubles, given the mean.
     */
    public static double standardDeviation(List<Double> data, double mean) {
        if (data.size() < 2) return 0.0;
        double sumSq = 0.0;
        for (double val : data) {
            sumSq += Math.pow(val - mean, 2);
        }
        // Sample standard deviation uses (n-1)
        return Math.sqrt(sumSq / (data.size() - 1));
    }


    /**
     * Concatenates multiple double arrays into one.
     */
    public static double[] concatArrays(double[]... arrays) {
        // int totalLength = 0;
        // for (double[] arr : arrays) {
        //     totalLength += arr.length;
        // }

        // double[] result = new double[totalLength];
        // int offset = 0;
        // for (double[] arr : arrays) {
        //     System.arraycopy(arr, 0, result, offset, arr.length);
        //     offset += arr.length;
        // }

        List<Double> result = new ArrayList<Double>();
        for (double[] arr: arrays) {
            for (int i=0;i<arr.length;i++) {
                result.add(arr[i]);
            }
        }
        double[] doubleResult = new double[result.size()];
        for (int j=0; j<result.size();j++) {
            doubleResult[j] = result.get(j);
        }
        return doubleResult;
    }
}
