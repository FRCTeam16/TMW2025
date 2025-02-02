package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Lifecycle;
import frc.robot.util.BSLogger;

import java.util.*;

public class VisionSubsystem extends SubsystemBase implements Lifecycle {
    private final Map<String, Limelight> limelightLookup = new HashMap<>();
    private final Limelight defaultLimelight;

    public VisionSubsystem(Iterable<Limelight> limelights) {
        Limelight tmpLimelight = null;
        for (Limelight limelight : limelights) {
            limelightLookup.put(limelight.getName(), limelight);
            if (tmpLimelight == null) {
                tmpLimelight = limelight;
            }
        }
        if (tmpLimelight != null) {
            this.defaultLimelight = tmpLimelight;
        } else {
            this.defaultLimelight = null;
            BSLogger.log("VisionSubsystem", "No Limelights were added to the VisionSubsystem");
        }
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public Optional<Limelight> getDefaultLimelight() {
        return Optional.ofNullable(defaultLimelight);
    }

    public List<Limelight> getLimelights() {
        return new ArrayList<>(limelightLookup.values());
    }

    public Limelight getLimelightByName(String name) {
        Limelight limelight = limelightLookup.get(name);
        if (limelight != null) {
            return limelight;
        } else {
            String message = "ERROR: No limelight named %s was added to the VisionSubsystem".formatted(name);
            DataLogManager.log(message);
            throw new IllegalArgumentException(message);
        }
    }

}
