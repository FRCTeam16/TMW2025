package frc.robot.subsystems.vision;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.VisionUtil;
import edu.wpi.first.wpilibj.DriverStation;

public class VisionAssist {
    private double x = 0;
    private double y = 0;
    private double rotation = 0;
    private boolean isRunning = false;
    private VisionUtil limeLight = new VisionUtil("LimeLight");

    private SwerveRequest.FieldCentric drive;

    public VisionAssist(SwerveRequest.FieldCentric drive){
        this.drive = drive;
        this.drive = drive;
    }

    public SwerveRequest deferDrive(Joystick driveStick, Joystick steerStick, double MaxSpeed, double MaxAngularRate){
        DriverStation.reportWarning("Defer Drive Reached", false);
        return drive.withVelocityX((-driveStick.getY() * MaxSpeed)) // Drive forward with negative Y (forward)
                .withVelocityY((-driveStick.getX() * MaxSpeed)) // Drive left with negative X (left)
                .withRotationalRate((-steerStick.getX() * MaxAngularRate)); // Drive counterclockwise with negative X (left)
    }

    public double getX(){
        if(isRunning)
            return x;
        return 0;
    }

    public double getY(){
        if(isRunning)
            return y;
        return 0;
    }

    public double getRotation(){
        if(isRunning)
            return rotation;
        return 0;
    }

    
    
}
