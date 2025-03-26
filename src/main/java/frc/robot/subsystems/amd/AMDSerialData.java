package frc.robot.subsystems.amd;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.BSLogger;

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
    private int coralScore = 0;
    private int elevatorLeftScore;
    private int elevatorRightScore;

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

    public byte getCoralScore() {
        return (byte) coralScore;
    }

    public void submitCoralScore(int coralScore) {
        BSLogger.log("AMDSerialData", "Coral Score: " + coralScore);
        SmartDashboard.putNumber("AMD/coral/intake", coralScore);
        this.coralScore = coralScore;
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
}
