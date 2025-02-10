package frc.robot.generated;

import java.util.HashMap;
import java.util.Map;

public class CanIDLookup {

    private final Map<String, Integer> canIDMap;

    private final Map<String, Integer> defaultCanIDs = Map.of(
            "algaeIntakeMotor", 52,
            "algaeArmMotor", 53,
            "coralIntakeTopMotor", 54,
            "coralIntakeBottomMotor", 55,
            "climberHand", 21,
            "climberPivot", 22,
            "elevatorLeftMotor", 30,
            "elevatorRightMotor", 31
    );


    private final Map<String, Integer> lowridaIDMap = Map.of();

    public CanIDLookup(RobotConfig.ConfigName config) {
        canIDMap = new HashMap<>(defaultCanIDs);
        if (config == RobotConfig.ConfigName.LOWRIDA) {
            canIDMap.putAll(lowridaIDMap);
        }
    }

    public int getCanID(String motorName) {
        return canIDMap.get(motorName);
    }
}
