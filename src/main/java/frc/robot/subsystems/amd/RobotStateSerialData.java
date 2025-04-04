package frc.robot.subsystems.amd;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Subsystems;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.vision.Limelight;

import java.util.Optional;

/**
 * This class is used to create status codes for the robot to send to the LED subsystem.
 */
public class RobotStateSerialData {
    private static final int NOCOMM_THRESHOLD = 4;
    private int lastComm = 0;
    private int noCommCounter = 0; // avoid intermittent counter by looking for a set number before reporting this


    public byte getRobotState() {
        // Communications status
        int robotState = 0;
        if (AMDSerialData.AMDPhase.Comm == Subsystems.ledSubsystem.getAMDSerialData().getCurrentPhase()) {
            if (DriverStation.isDisabled()) {
                robotState = 1;
            } else if (DriverStation.isAutonomous()) {
                robotState = 2;
            } else if (DriverStation.isTeleop()) {
                robotState = 3;
            }
            if (robotState == 0) {
                noCommCounter++;
                robotState = (noCommCounter < NOCOMM_THRESHOLD) ? lastComm : 0;
            } else {
                noCommCounter = 0;
            }
            lastComm = robotState;
        } else {
            robotState = Subsystems.ledSubsystem.getAMDSerialData().getCurrentPhase().phaseNumber;
        }
        return (byte) robotState;
    }

    public byte getAllianceColor() {
        byte allianceColor = 0;
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                allianceColor = 1;
            } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                allianceColor = 2;
            }
        }
        return allianceColor;
    }

    public byte getHasPartCode() {
        // 0 - no part, 1 - coral, 2- algae, 3 - both
        int hasPartCode = (Subsystems.coralIntake.coralDetectedAtBottomSensor() ? 1 : 0) +
                (Subsystems.algaeIntake.isAlgaeDetected() ? 2 : 0) ;
        return (byte) hasPartCode;
    }

    public byte getElevatorCode() {
        // 0 - down
        // 1 - going to L2
        // 2 - going to L3
        // 3 - going to L4
        // 4 - in position at l2
        // 5 - in position at l3
        // 6 - in position at l4
        // 7 - going to alga low
        // 8 - going to alga high
        // 9 - in position alga low
        // 10 - in position alga high
        Elevator.ElevatorSetpoint requestedSetpoint = Subsystems.elevator.getRequestedSetpoint();
        boolean elevatorInPosition = Subsystems.elevator.isInPosition();
        int elevatorStatus = switch (requestedSetpoint) {
            case Zero -> 0;
            case TROUGH -> 0;
            case L2 -> elevatorInPosition ? 4 : 1;
            case L3 -> elevatorInPosition ? 5 : 2;
            case L4 -> elevatorInPosition ? 6 : 3;
            case AlgaeBarge -> 0;
            case AlgaeProcessor -> 0;
            case AlgaeReefHigh -> elevatorInPosition ? 10 : 8;
            case AlgaeReefLow -> elevatorInPosition ? 9 : 7;
        };
        return(byte) elevatorStatus;
    }

    public boolean getAprilTagDistanceInThreshold() {
        return Subsystems.visionOdometryUpdater.getTargetDistance()
                .map(d -> MathUtil.isNear(Math.abs(d), 0.406, 0.05))
                .orElse(false);
    }


    public byte getAprilTagAngle() {
        Optional<Boolean> targetAngle = Subsystems.visionSubsystem.getDefaultLimelight()
                .map(Limelight::getTargetInfo)
                .map(info -> {
                    if (info.hasTarget()) {
                        if (info.xOffset() < 0) {
                            return info.xOffset() >= -27.2 && info.xOffset() <= -19.5;
                        } else {
                            return info.xOffset() >= 18.50 && info.xOffset() <= 26.25;
                        }
                    } else {
                        return false;
                    }
                });
        return targetAngle.orElse(false) ? (byte) 1 : (byte) 0;
    }
}
