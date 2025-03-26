import frc.robot.subsystems.amd.AMDStats;
import frc.robot.subsystems.amd.DriveInfo;
import frc.robot.subsystems.amd.SwerveDataCollector;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

public class TestMADOutlierDetection {

    public Map<String, List<Double>> parseCSV(String dataFilePath) throws IOException {
        List<String> lines = Files.readAllLines(Paths.get(dataFilePath));
        String[] headers = lines.get(0).split(",");
        java.util.Map<String, java.util.List<Double>> columnData = new java.util.HashMap<>();
        for (String header : headers) {
            columnData.put(header, new java.util.ArrayList<>());
        }
        // skip header
        for (int i = 1; i < lines.size(); i++) {
            String line = lines.get(i);
            String[] columns = line.split(",");
            for (int j = 0; j < columns.length; j++) {
                double value = Double.parseDouble(columns[j]);
                columnData.get(headers[j]).add(value);
            }
            // ...use or print the values...
//            System.out.println("Row " + i + ": " + java.util.Arrays.toString(columns));
        }
        System.out.println("First Column: " + columnData.get(headers[0]));
        return columnData;
    }

    @Test
    public void testSingleMADOutlier() {
        String dataFilePath = "src/test/testdata/current.csv";
        try {
            Map<String, List<Double>> columnData = parseCSV(dataFilePath);

            List<Double> FL = columnData.get("FL").stream().mapToDouble(Double::doubleValue).boxed().toList();
            List<Double> outliers = AMDStats.detectOutliersMAD(FL, 3.5);
            System.out.println("Outliers: " + outliers);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Test
    public void testSingleZScoreOutlier() {
        String dataFilePath = "src/test/testdata/current.csv";
        try {
            Map<String, List<Double>> columnData = parseCSV(dataFilePath);

            List<Double> FL = columnData.get("FL").stream().mapToDouble(Double::doubleValue).boxed().toList();
            List<Double> outliers = AMDStats.detectOutliersZScore(FL, 3);
            assertEquals(3, outliers.size());
            System.out.println("Outliers: " + outliers);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Test
    public void testAllMADOutlier() {
        String dataFilePath = "src/test/testdata/current.csv";
        try {
            Map<String, List<Double>> columnData = parseCSV(dataFilePath);
            SwerveDataCollector collector = new SwerveDataCollector();

            double[] FL = columnData.get("FL").stream().mapToDouble(Double::doubleValue).toArray();
            double[] FR = columnData.get("FR").stream().mapToDouble(Double::doubleValue).toArray();
            double[] RL = columnData.get("RL").stream().mapToDouble(Double::doubleValue).toArray();
            double[] RR = columnData.get("RR").stream().mapToDouble(Double::doubleValue).toArray();
            DriveInfo<Boolean> outliers = AMDStats.detectOutliers(FL, FR, RL, RR, "test");
            System.out.println("Outliers: " + outliers);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
