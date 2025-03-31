package frc.robot.subsystems.amd;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.util.BSLogger;

import java.util.Optional;

/**
 * This class is used to store the data that is sent during the AMD process.
 */
public class AMDSerialData {

    public enum AMDPhase {
        Comm(0),
        Drivetrain(4),
        Elevator(5),
        CoralShooter(6),
        AlgaeIntake(7),
        Climber(8),
        AMDEnd(9);

        public final int phaseNumber;

        AMDPhase(int phaseNumber) {
            this.phaseNumber = phaseNumber;
        }
    }

    private AMDSerialData.AMDPhase currentPhase = AMDSerialData.AMDPhase.Comm;
    private DriveInfo<Integer> driveScores = new DriveInfo<>(0);
    private DriveInfo<Integer> steerScores = new DriveInfo<>(0);
    private int leftCoralScore = 0;
    private int rightCoralScore = 0;
    private int elevatorLeftScore;
    private int elevatorRightScore;
    private int algaeArmScore = 0;
    private int algaeIntakeScore = 0;

    public void startAMDPhase(AMDPhase amdPhase) {
        this.currentPhase = amdPhase;
    }

    public AMDPhase getCurrentPhase() {
        return currentPhase;
    }

    public DriveInfo<Integer> getDriveScores() {
        return driveScores;
    }

    public void submitDriveDMSScores(DriveInfo<Integer> driveScores) {
        BSLogger.log("LEDSubsystem", "AMD Drive Scores");
        BSLogger.log("LEDSubsystem", "FL: " + driveScores.FL);
        BSLogger.log("LEDSubsystem", "FR: " + driveScores.FR);
        BSLogger.log("LEDSubsystem", "RL: " + driveScores.RL);
        BSLogger.log("LEDSubsystem", "RR: " + driveScores.RR);

        SmartDashboard.putNumber("AMD/swerve/driveFL", driveScores.FL);
        SmartDashboard.putNumber("AMD/swerve/driveFR", driveScores.FR);
        SmartDashboard.putNumber("AMD/swerve/driveRL", driveScores.RL);
        SmartDashboard.putNumber("AMD/swerve/driveRR", driveScores.RR);

        this.driveScores = driveScores;
    }

    public DriveInfo<Integer> getSteerScores() {
        return steerScores;
    }

    public void submitSteerDMSScores(DriveInfo<Integer> steerScores) {
        BSLogger.log("AMDSerialData", "AMD Steer Scores");
        BSLogger.log("AMDSerialData", "FL: " + steerScores.FL);
        BSLogger.log("AMDSerialData", "FR: " + steerScores.FR);
        BSLogger.log("AMDSerialData", "RL: " + steerScores.RL);
        BSLogger.log("AMDSerialData", "RR: " + steerScores.RR);

        SmartDashboard.putNumber("AMD/swerve/steerFL", steerScores.FL);
        SmartDashboard.putNumber("AMD/swerve/steerFR", steerScores.FR);
        SmartDashboard.putNumber("AMD/swerve/steerRL", steerScores.RL);
        SmartDashboard.putNumber("AMD/swerve/steerRR", steerScores.RR);

        this.steerScores = steerScores;
    }


    public void resetDMSScores() {
        this.driveScores = new DriveInfo<>(0);
        this.steerScores = new DriveInfo<>(0);
    }

    public byte getLeftCoralScore() {
        return (byte) leftCoralScore;
    }
    public byte getRightCoralScore() {
        return (byte) rightCoralScore;
    }

    public void submitLeftCoralScore(int coralScore) {
        BSLogger.log("AMDSerialData", "Left Coral Score: " + coralScore);
        SmartDashboard.putNumber("AMD/coral/left_intake", coralScore);
        this.leftCoralScore = coralScore;
    }
    public void submitRightCoralScore(int coralScore) {
        BSLogger.log("AMDSerialData", "Right Coral Score: " + coralScore);
        SmartDashboard.putNumber("AMD/coral/right_intake", coralScore);
        this.rightCoralScore = coralScore;
    }

    public byte getElevatorLeftScore() {
        return (byte) elevatorLeftScore;
    }

    public byte getElevatorRightScore() {
        return (byte) elevatorRightScore;
    }

    public void submitElevatorScore(int leftScore, int rightScore) {
        BSLogger.log("AMDSerialData", "Elevator Left Score: " + leftScore + " | Right Score: " + rightScore);
        SmartDashboard.putNumber("AMD/elevator/left", leftScore);
        SmartDashboard.putNumber("AMD/elevator/right", rightScore);
        this.elevatorLeftScore = leftScore;
        this.elevatorRightScore = rightScore;
    }

    public byte getAlgaeArmScore() {
        return (byte) algaeArmScore;
    }

    public void submitAlgaeArmScore(int score) {
        BSLogger.log("AMDSerialData", "Algae Arm Score: " + score);
        SmartDashboard.putNumber("AMD/algae_arm", score);
        this.algaeArmScore = score;
    }

    public byte getAprilTagAngle() {
        Optional<Double> targetAngle = Subsystems.visionSubsystem.getDefaultLimelight()
                .map(Limelight::getTargetInfo)
                .map(info -> {
                    if (info.hasTarget()) {
                        return info.xOffset();
                    } else {
                        return -99.0;
                    }
                });
        return targetAngle.orElse(-99.0).byteValue();
    }

    public byte getAlgaeIntakeScore() {
        return (byte) algaeIntakeScore;
    }

    public void submitAlgaeIntakeScore(int score) {
        BSLogger.log("AMDSerialData", "Algae Intake Score: " + score);
        SmartDashboard.putNumber("AMD/algae_intake", score);
        this.algaeIntakeScore = score;
    }

    public boolean getAprilTagDistanceInThreshold() {
        return Subsystems.visionOdometryUpdater.getTargetDistance()
                .map(d -> d < 0.4)
                .orElse(false);
    }
}
