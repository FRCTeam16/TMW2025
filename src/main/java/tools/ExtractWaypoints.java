package tools;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class ExtractWaypoints {

    public static void main(String[] args) {
        try {
            // Read the JSON file
            ObjectMapper mapper = new ObjectMapper();
            JsonNode root = mapper.readTree(new File("src/main/deploy/pathplanner/paths/BlueCoordPath.path"));

            // Get the waypoints array
            ArrayNode waypoints = (ArrayNode) root.get("waypoints");

            // Create a HashMap to store the extracted data
            Map<String, double[]> waypointMap = new HashMap<>();

            // Extract the anchor x, y values and linkedName
            for (JsonNode waypoint : waypoints) {
                String linkedName = waypoint.get("linkedName").asText(null);
                if (linkedName != null) {
                    JsonNode anchor = waypoint.get("anchor");
                    double x = anchor.get("x").asDouble();
                    double y = anchor.get("y").asDouble();
                    waypointMap.put(linkedName, new double[]{x, y});
                }
            }

            // Print the HashMap
            waypointMap.keySet().stream().sorted().forEach(key -> {
                var value = waypointMap.get(key);
                System.out.println("lookup.put(\"" + key + "\", new Translation2d(" + value[0] + ", " + value[1] + "));");
            });

        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}