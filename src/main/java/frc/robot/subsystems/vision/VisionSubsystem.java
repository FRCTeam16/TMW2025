package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Lifecycle;
import frc.robot.util.BSLogger;

import java.util.*;

import static edu.wpi.first.units.Units.Seconds;


public class VisionSubsystem extends SubsystemBase implements Lifecycle {
    private final Map<String, Limelight> limelightLookup = new HashMap<>();
    private final Limelight defaultLimelight;
    private final Timer updateTimer = new Timer();

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
        updateTimer.start();
    }

    @Override
    public void robotInit() {
        selectPipeline(Pipeline.View);
    }

    @Override
    public void teleopInit() {
        selectPipeline(Pipeline.April);
    }

    @Override
    public void autoInit() {
        selectPipeline(Pipeline.April);
    }


    @Override
    public void periodic() {
        if (updateTimer.hasElapsed(1.0)) {
            getActiveLimelightName();
            updateTimer.reset();
        }
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

    /**
     * Determines the best target info from all limelights
     *
     * @return target info
     */
    public Optional<VisionTypes.TargetInfo> getTargetInfo() {
        return getLimelights().stream()
                .map(Limelight::getTargetInfo)
                .max(Comparator.comparing(VisionTypes.TargetInfo::targetArea));

    }

    public void selectPipeline(Pipeline pipeline) {
        getLimelights().forEach(limelight -> limelight.setPipeline(pipeline));
    }

    public void resetIDFilter() {
        getLimelights().forEach(limelight -> LimelightHelpers.SetFiducialIDFiltersOverride(limelight.getName(), new int[] {}));
    }

    // Robot specific

    public String getActiveLimelightName() {
        String active = (Elevator.ElevatorSetpoint.Zero == Subsystems.elevator.getRequestedSetpoint() ? "limelight" : "limelight-base");
        SmartDashboard.putString("Subsystems/Vision/activeLimelight", active);
        return active;
    }

    public Limelight getActiveLimelight() {
        return getLimelightByName(getActiveLimelightName());
    }
}
