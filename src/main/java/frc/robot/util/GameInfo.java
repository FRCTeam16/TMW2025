package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.Optional;

public class GameInfo {
    public static boolean isRedAlliance() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
    }

    public static boolean isBlueAlliance() {
        return !isRedAlliance();
    }
}
