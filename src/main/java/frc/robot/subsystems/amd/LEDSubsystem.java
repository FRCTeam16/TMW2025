package frc.robot.subsystems.amd;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Parity;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Lifecycle;

public class LEDSubsystem extends SubsystemBase implements Lifecycle {
    private static final int BUFFER_SIZE = 26;
    private boolean running = true;
    private SerialPort serial;
    private int secondsToClimb = 30;

    private final RobotStateSerialData robotStateSerialData = new RobotStateSerialData();
    private final AMDSerialData amdSerialData = new AMDSerialData();

    /**
     * Creates a new LEDSubsystem.
     */
    public LEDSubsystem() {
        SmartDashboard.setDefaultNumber("LEDClimbTime", 30);
        try {
            if (running) {
                serial = new SerialPort(57600, SerialPort.Port.kUSB1, 8, Parity.kNone, StopBits.kOne);
                serial.setWriteBufferSize(BUFFER_SIZE);
                serial.setWriteBufferMode(WriteBufferMode.kFlushWhenFull);
            }
        } catch (Exception e) {
            System.err.println("Unable to create DMS/LED subsystem, problem with serial port: " + e.getMessage());
            running = false;
        }
    }

    /**
     * Called out of band by a scheduled periodic in main robot
     */
    public void Report() {
        SmartDashboard.putBoolean("DMS/Running", running);
        SmartDashboard.putBoolean("DMS/HasSerial", (serial != null));
        if (running && serial != null) {
            try {
                SendData(new DriveInfo<>(0.0), new DriveInfo<>(0.0));
            } catch (Exception e) {
                // error sending data
                System.out.println("LED EXCEPTION: " + e.getMessage());
                running = false;
            } catch (Error e) {
                System.out.println("LED ERROR: " + e.getMessage());
                running = false;
            }
        }
    }

    public RobotStateSerialData getRobotStateSerialData() {
        return robotStateSerialData;
    }

    public AMDSerialData getAMDSerialData() {
        return amdSerialData;
    }

    public void SendData(DriveInfo<Double> driveMotor, DriveInfo<Double> steerMotor) {

        byte[] buffer = new byte[BUFFER_SIZE];

        buffer[0] = (byte) 254;
        buffer[1] = robotStateSerialData.getRobotState(); // comm status
        buffer[2] = robotStateSerialData.getAllianceColor();
        buffer[3] = robotStateSerialData.getHasPartCode();
        buffer[4] = robotStateSerialData.getElevatorCode();

        // DMS info
        buffer[5] = amdSerialData.getDriveScores().FL.byteValue();
        buffer[6] = amdSerialData.getSteerScores().FL.byteValue();
        buffer[7] = amdSerialData.getDriveScores().FR.byteValue();
        buffer[8] = amdSerialData.getSteerScores().FR.byteValue();
        buffer[9] = amdSerialData.getDriveScores().RL.byteValue();
        buffer[10] = amdSerialData.getSteerScores().RL.byteValue();
        buffer[11] = amdSerialData.getDriveScores().RR.byteValue();
        buffer[12] = amdSerialData.getSteerScores().RR.byteValue();

        buffer[13] = (byte) 0; // AMD climber
        buffer[14] = (byte) amdSerialData.getElevatorLeftScore(); // AMD elevatorLeft
        buffer[15] = (byte) amdSerialData.getElevatorRightScore(); // AMD elevatorRight
        buffer[16] = (byte) amdSerialData.getLeftCoralScore(); // AMD coral shooter
        buffer[17] = (byte) amdSerialData.getRightCoralScore(); // AMD coralRight
        buffer[18] = (byte) amdSerialData.getAlgaeIntakeScore(); // AMD algae intake
        buffer[19] = (byte) amdSerialData.getAlgaeArmScore(); // AMD algaePivotMotor
        buffer[20] = (byte) amdSerialData.getAprilTagAngle(); // April Tag Angle
        buffer[21] = (byte) 0; // extra
        buffer[22] = (byte) 0; // barge distance
        buffer[23] = (byte) (amdSerialData.getAprilTagDistanceInThreshold() ? 1 : 0); // April Tag Distance
        buffer[24] = (byte) 0; // extra
        buffer[25] = (byte) 255;

        this.serial.write(buffer, buffer.length);
        this.serial.flush();
    }

    @Override
    public void periodic() {

        int climbTimeFromDashboard = (int) SmartDashboard.getNumber("LEDClimbTime", 30);
        if (climbTimeFromDashboard != secondsToClimb) {
            secondsToClimb = climbTimeFromDashboard;
        }
    }
}
