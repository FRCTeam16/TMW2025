package frc.robot.generated;

import java.util.HashMap;
import java.util.Map;

/**
 * This class is used to look up the non-drivetrain CAN IDs for the motors on the robot. This is useful for
 * configuring the robot for different configurations.
 * <p>
 * In order to add a new motor to the lookup, add the motor name and CAN ID to the defaultCanIDs map.
 * In order to override a CAN ID for a specific configuration, add the motor name and CAN ID to the
 * configuration specific map.
 */
public class CanIDLookup {

    private final Map<String, Integer> canIDMap;

    private final Map<String, Integer> defaultCanIDs = Map.of(
            "algaeIntakeMotor", 52,
            "algaeArmMotor", 53,
            "coralIntakeLeftMotor", 54,
            "coralIntakeRightMotor", 55,
            "climberMotor", 21,
            "elevatorLeftMotor", 30,
            "elevatorRightMotor", 31,
            "funnelPivotMotor", 32,
            "funnelConveyorMotor", 33
    );


    private final Map<String, Integer> lowridaIDMap = Map.of();

    public CanIDLookup(RobotConfig.ConfigName config) {
        canIDMap = new HashMap<>(defaultCanIDs);
        if (config == RobotConfig.ConfigName.LOWRIDA) {
            canIDMap.putAll(lowridaIDMap);
        }
    }


    public int getCanID(String motorName) {
        if (!canIDMap.containsKey(motorName)) {
            throw new IllegalArgumentException("No CAN ID found for motor: " + motorName);
        }
        return canIDMap.get(motorName);
    }
}
