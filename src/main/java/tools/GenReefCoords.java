package tools;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.wpi.first.math.geometry.Translation2d;

import java.io.File;
import java.util.HashMap;
import java.util.Map;

public class GenReefCoords {

    private static Map<String, Translation2d> getNewWaypoints() {
        Map<String, Translation2d> lookup = new HashMap<>();

        lookup.put("BlueA", new Translation2d(2.69, 4.42));
        lookup.put("BlueB", new Translation2d(2.69, 4.08));
        lookup.put("BlueC", new Translation2d(3.6874999999998677, 2.9312499999954924));
        lookup.put("BlueD", new Translation2d(4.031249999999868, 2.7749999999954924));
        lookup.put("BlueE", new Translation2d(4.981249999999868, 2.6749999999954923));
        lookup.put("BlueF", new Translation2d(5.281249999999867, 2.9312499999954924));
        lookup.put("BlueG", new Translation2d(5.8326634106272275, 3.8437499999954925));
        lookup.put("BlueH", new Translation2d(5.8326634106272275, 4.15));
        lookup.put("BlueI", new Translation2d(5.281249999999867, 5.118749999995287));
        lookup.put("BlueJ", new Translation2d(4.981249999999868, 5.343750000003217));
        lookup.put("BlueL", new Translation2d(4.031249999999868, 5.343750000003217));

        return lookup;
    }

    public static void main(String[] args) {
        try {
            // Read the JSON file
            ObjectMapper mapper = new ObjectMapper();
            ObjectNode root = (ObjectNode) mapper.readTree(new File("src/main/deploy/pathplanner/paths/BlueCoordPath.path"));

            // Get the waypoints array
            ArrayNode waypoints = (ArrayNode) root.get("waypoints");

            // Get the new waypoint values
            var newWaypoints = getNewWaypoints();

            // Modify the waypoints
            for (int i = 0; i < waypoints.size(); i++) {
                ObjectNode waypoint = (ObjectNode) waypoints.get(i);
                JsonNode linkedNameNode = waypoint.get("linkedName");
                String linkedName = null;
                if (linkedNameNode != null && !linkedNameNode.isNull()) {
                    linkedName = linkedNameNode.asText();
                } else {
                    continue;
                }

                Translation2d newPoint = newWaypoints.get(linkedName);
                if (newPoint != null) {
                    ObjectNode anchor = (ObjectNode) waypoint.get("anchor");
                    anchor.put("x", newPoint.getX()); // Set your desired x value
                    anchor.put("y", newPoint.getY()); // Set your desired y value
                }
            }


            // Write the modified JSON back to the file
            String timestamp = java.time.format.DateTimeFormatter.ofPattern("yyyy-MM-dd'T'HH-mm-ss").format(java.time.LocalDateTime.now());//
            new File("src/main/deploy/pathplanner/paths/generated").mkdirs();
            mapper.writerWithDefaultPrettyPrinter().writeValue(new File("src/main/deploy/pathplanner/paths/generated/BlueCoordPath_" + timestamp + ".path"), root);
            mapper.writerWithDefaultPrettyPrinter().writeValue(System.out, root);

        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}