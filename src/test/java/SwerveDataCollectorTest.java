import frc.robot.subsystems.DMS.DriveInfo;
import frc.robot.subsystems.DMS.SwerveDataCollector;
import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

public class SwerveDataCollectorTest {
    // ...existing code...

    @Test
    public void testNoOutliers() {
        SwerveDataCollector collector = new SwerveDataCollector();
        List<Double> velocityData1 = Arrays.asList(1490.0, 1500.0, 1510.0, 1505.0, 1495.0, 1498.0, 1502.0, 1507.0);
        List<Double> velocityData2 = Arrays.asList(1485.0, 1495.0, 1505.0, 1515.0, 1500.0, 1492.0, 1508.0, 1503.0);
        List<Double> velocityData3 = Arrays.asList(1500.0, 1500.0, 1500.0, 1505.0, 1495.0, 1497.0, 1503.0, 1501.0);
        List<Double> velocityData4 = Arrays.asList(1490.0, 1500.0, 1510.0, 1500.0, 1500.0, 1496.0, 1504.0, 1499.0);
        List<Double> currentData1 = Arrays.asList(2.9, 3.0, 3.1, 3.05, 2.95, 3.02, 2.98, 3.03);
        List<Double> currentData2 = Arrays.asList(2.8, 3.1, 3.0, 3.2, 2.9, 3.05, 2.97, 3.01);
        List<Double> currentData3 = Arrays.asList(3.0, 3.0, 3.0, 3.05, 2.95, 3.04, 2.96, 3.02);
        List<Double> currentData4 = Arrays.asList(2.95, 3.05, 3.1, 3.0, 2.9, 3.03, 2.97, 3.01);

        double[] vd1 = velocityData1.stream().mapToDouble(Double::doubleValue).toArray();
        double[] vd2 = velocityData2.stream().mapToDouble(Double::doubleValue).toArray();
        double[] vd3 = velocityData3.stream().mapToDouble(Double::doubleValue).toArray();
        double[] vd4 = velocityData4.stream().mapToDouble(Double::doubleValue).toArray();

        // Debug statements
        System.out.println("Test No Outliers - Velocity Medians:");
        System.out.println("FL: " + SwerveDataCollector.median(vd1));
        System.out.println("FR: " + SwerveDataCollector.median(vd2));
        System.out.println("RL: " + SwerveDataCollector.median(vd3));
        System.out.println("RR: " + SwerveDataCollector.median(vd4));

        DriveInfo<Boolean> result = SwerveDataCollector.detectOutliers(vd1, vd2, vd3, vd4);
        assertFalse(result.FL);
        assertFalse(result.FR);
        assertFalse(result.RL);
        assertFalse(result.RR);

        result = SwerveDataCollector.detectOutliers(
                currentData1.stream().mapToDouble(Double::doubleValue).toArray(),
                currentData2.stream().mapToDouble(Double::doubleValue).toArray(),
                currentData3.stream().mapToDouble(Double::doubleValue).toArray(),
                currentData4.stream().mapToDouble(Double::doubleValue).toArray()
        );
        assertFalse(result.FL);
        assertFalse(result.FR);
        assertFalse(result.RL);
        assertFalse(result.RR);
    }

    @Test
    public void testOneOutlier() {
        SwerveDataCollector collector = new SwerveDataCollector();
        List<Double> velocityData1 = Arrays.asList(
                1500.0, 1495.0, 1505.0, 1497.0, 1503.0, 1498.0, 1502.0, 1496.0, 1504.0, 1499.0,
                1501.0, 1494.0, 1506.0, 1500.0, 1495.5, 1504.2, 1497.8, 1503.1, 1498.4, 1502.6,
                1501.2, 1499.5, 1500.4, 1500.8, 1499.1, 1502.9, 1498.3, 1504.5, 1496.9, 1503.7,
                1497.6, 1502.3, 1495.2, 1501.8, 1499.7, 1500.3, 1498.2, 1504.8, 1496.4, 1503.9,
                1497.2, 1501.5, 1502.1, 1498.6, 1500.9, 1496.6, 1503.0, 1499.8, 1501.3, 1502.7
        );

        List<Double> velocityData2 = Arrays.asList(
                1501.0, 1499.0, 1500.5, 1498.0, 1502.5, 1497.5, 1503.5, 1500.2, 1498.3, 1501.7,
                1499.8, 1501.2, 1497.9, 1502.1, 1498.4, 1500.3, 1499.2, 1502.0, 1498.6, 1503.0,
                1497.3, 1502.8, 1500.9, 1499.5, 1501.6, 1502.3, 1497.2, 1498.7, 1503.2, 1500.1,
                1499.4, 1501.1, 1502.6, 1498.8, 1499.9, 1500.6, 1497.8, 1502.9, 1499.1, 1503.3,
                1498.1, 1500.8, 1497.0, 1503.7, 1499.6, 1501.4, 1502.2, 1498.9, 1500.7, 1497.6
        );

        List<Double> velocityData3 = Arrays.asList(
                1499.5, 1500.0, 1500.1, 1500.2, 1498.7, 1502.0, 1501.3, 1497.6, 1503.2, 1498.3,
                1499.8, 1502.4, 1500.9, 1497.7, 1503.1, 1498.2, 1499.7, 1501.1, 1502.7, 1498.4,
                1499.9, 1500.4, 1497.9, 1502.6, 1501.6, 1498.6, 1499.2, 1503.3, 1497.4, 1501.8,
                1500.3, 1498.8, 1502.9, 1497.3, 1499.6, 1503.0, 1500.7, 1497.5, 1501.9, 1498.5,
                1499.3, 1502.5, 1497.2, 1500.6, 1501.2, 1498.1, 1501.7, 1499.1, 1502.8, 1497.8
        );

        List<Double> velocityData4 = Arrays.asList(
                // First 25 items around 1500
                1498.0, 1501.0, 1499.5, 1502.5, 1497.5, 1503.0, 1500.6, 1498.6, 1502.9, 1497.9,
                1499.9, 1502.4, 1498.8, 1500.2, 1497.1, 1503.2, 1499.4, 1501.1, 1498.3, 1502.1,
                1497.8, 1503.1, 1499.0, 1501.5, 1498.7,
                // Next 25 items significantly larger
                3000.0, 3250.0, 3400.0, 3100.0, 3500.0, 2999.0, 3800.0, 3600.0, 4000.0, 4500.0,
                3200.0, 3300.0, 3100.0, 3700.0, 3900.0, 4100.0, 4200.0, 4400.0, 4600.0, 4700.0,
                4800.0, 4900.0, 5000.0, 5300.0, 5500.0
        );

        double[] vd1 = velocityData1.stream().mapToDouble(Double::doubleValue).toArray();
        double[] vd2 = velocityData2.stream().mapToDouble(Double::doubleValue).toArray();
        double[] vd3 = velocityData3.stream().mapToDouble(Double::doubleValue).toArray();
        double[] vd4 = velocityData4.stream().mapToDouble(Double::doubleValue).toArray();

        DriveInfo<Boolean> result = SwerveDataCollector.detectOutliers(vd1, vd2, vd3, vd4);
        System.out.println("Result: " + result.FL + ", " + result.FR + ", " + result.RL + ", " + result.RR);

        assertFalse(result.FL);
        assertFalse(result.FR);
        assertFalse(result.RL);
        assertTrue(result.RR);
    }

    @Test
    public void testTwoOutliers() {
        SwerveDataCollector collector = new SwerveDataCollector();
        List<Double> velocityData1 = Arrays.asList(
                1501.0, 1499.0, 1500.5, 1498.0, 1502.5, 1497.5, 1503.5, 1500.2, 1498.3, 1501.7,
                1499.8, 1501.2, 1497.2, 1502.1, 1498.9, 1500.3, 1499.2, 1502.0, 1498.6, 1503.0,
                1499.0, 1500.7, 1497.6, 1498.1, 1502.3, 1499.4, 1501.6, 1497.8, 1503.7, 1500.8
        );

        List<Double> velocityData2 = Arrays.asList(
                // First 15 items around 1500
                1498.0, 1501.0, 1499.5, 1502.5, 1497.5, 1500.6, 1498.6, 1502.9, 1497.9, 1499.9,
                1502.4, 1498.8, 1500.2, 1497.1, 1499.4,
                // Next 15 items significantly larger
                3000.0, 3400.0, 3100.0, 3500.0, 3800.0, 4000.0, 4200.0, 4400.0, 4600.0, 4700.0,
                4800.0, 4900.0, 5000.0, 5300.0, 5500.0
        );

        List<Double> velocityData3 = Arrays.asList(
                1499.5, 1500.0, 1500.1, 1500.2, 1498.7, 1502.0, 1501.3, 1497.6, 1503.2, 1498.3,
                1502.4, 1498.2, 1499.7, 1501.1, 1498.4, 1499.9, 1500.4, 1497.9, 1502.6, 1501.6,
                1498.6, 1499.2, 1503.3, 1497.4, 1500.3, 1499.8, 1501.9, 1497.8, 1502.8, 1499.3
        );

        List<Double> velocityData4 = Arrays.asList(
                // First 15 items around 1500
                1498.0, 1501.0, 1499.5, 1502.5, 1497.5, 1500.6, 1498.6, 1502.9, 1497.9, 1499.9,
                1502.4, 1498.8, 1500.2, 1497.1, 1499.4,
                // Next 15 items significantly larger
                3000.0, 3400.0, 3100.0, 3500.0, 3800.0, 4000.0, 4200.0, 4400.0, 4600.0, 4700.0,
                4800.0, 4900.0, 5000.0, 5300.0, 5500.0
        );

        List<Double> currentData1 = Arrays.asList(
                2.9, 3.0, 3.1, 3.05, 2.95, 3.02, 2.98, 3.03, 3.01, 3.04,
                2.99, 2.85, 3.06, 2.95, 3.00, 3.02, 2.97, 3.05, 2.96, 3.01,
                3.10, 2.90, 3.04, 2.99, 3.02, 3.03, 2.87, 2.98, 3.01, 3.00
        );

        List<Double> currentData2 = Arrays.asList(
                2.8, 3.1, 3.0, 3.2, 2.9, 3.05, 2.97, 3.01, 2.85, 3.05,
                3.09, 2.93, 3.02, 3.07, 2.99, 2.95, 3.06, 2.88, 3.10, 3.00,
                2.99, 2.96, 3.04, 2.86, 3.03, 3.02, 2.90, 2.94, 3.05, 2.97
        );

        List<Double> currentData3 = Arrays.asList(
                // First 15 items around 3.0
                2.95, 3.05, 3.1, 3.0, 2.9, 3.03, 2.97, 2.98, 3.06, 2.96,
                3.01, 3.04, 2.94, 3.07, 2.99,
                // Next 15 items significantly larger
                15.0, 20.0, 25.0, 28.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0,
                60.0, 65.0, 70.0, 75.0, 80.0
        );

        List<Double> currentData4 = Arrays.asList(
                // First 15 items around 3.0
                2.95, 3.05, 3.1, 3.0, 2.9, 3.03, 2.97, 2.98, 3.06, 2.96,
                3.01, 3.04, 2.94, 3.07, 2.99,
                // Next 15 items significantly larger
                15.0, 20.0, 25.0, 28.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0,
                60.0, 65.0, 70.0, 75.0, 80.0
        );

        double[] vd1 = velocityData1.stream().mapToDouble(Double::doubleValue).toArray();
        double[] vd2 = velocityData2.stream().mapToDouble(Double::doubleValue).toArray();
        double[] vd3 = velocityData3.stream().mapToDouble(Double::doubleValue).toArray();
        double[] vd4 = velocityData4.stream().mapToDouble(Double::doubleValue).toArray();

        DriveInfo<Boolean> velocityResult = SwerveDataCollector.detectOutliers(vd1, vd2, vd3, vd4);
        assertFalse(velocityResult.FL);
        assertTrue(velocityResult.FR);
        assertFalse(velocityResult.RL);
        assertTrue(velocityResult.RR);

        double[] cd1 = currentData1.stream().mapToDouble(Double::doubleValue).toArray();
        double[] cd2 = currentData2.stream().mapToDouble(Double::doubleValue).toArray();
        double[] cd3 = currentData3.stream().mapToDouble(Double::doubleValue).toArray();
        double[] cd4 = currentData4.stream().mapToDouble(Double::doubleValue).toArray();

        DriveInfo<Boolean> currentResult = SwerveDataCollector.detectOutliers(cd1, cd2, cd3, cd4);
        assertFalse(currentResult.FL);
        assertFalse(currentResult.FR);
        assertTrue(currentResult.RL);
        assertTrue(currentResult.RR);

        DriveInfo<Integer> score = new DriveInfo<>(0);
        score.FL = SwerveDataCollector.calculateStatus(currentResult.FL, velocityResult.FL);
        score.FR = SwerveDataCollector.calculateStatus(currentResult.FR, velocityResult.FR);
        score.RL = SwerveDataCollector.calculateStatus(currentResult.RL, velocityResult.RL);
        score.RR = SwerveDataCollector.calculateStatus(currentResult.RR, velocityResult.RR);
        assertEquals(0, score.FL);
        assertEquals(1, score.FR);  // outlier in current
        assertEquals(2, score.RL);
        assertEquals(3, score.RR);
    }
}
