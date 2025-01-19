package frc.robot.subsystems.vision;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.hci.SwerveSupplier;
import frc.robot.util.PIDHelper;
import frc.robot.util.VisionUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.units.measure.LinearVelocity;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class VisionAssist {
    private double distance = 0;
    private VisionUtil limeLight = new VisionUtil("LimeLight");
    double tagAngle;
    double robotAngle;
    private double robotCentricX = 0;
    private double robotCentricY = 0;
    private double fieldCentricX = 0;
    private double fieldCentricY = 0;

    private PIDHelper speedHelper = new PIDHelper("VisionAssist", true);

    private SwerveRequest.FieldCentric drive;

    public VisionAssist(SwerveRequest.FieldCentric drive) {
        this.drive = drive;
        speedHelper.initialize(0.05, 0, 0,0 ,0, 0);
    }

    public SwerveRequest deferDrive(SwerveSupplier swerveSupplier){
        DriverStation.reportWarning("Defer Drive Reached", false);
        return drive.withVelocityX(swerveSupplier.supplyX().plus(MetersPerSecond.of(fieldCentricX))) // Drive forward with negative Y (forward) + (distance * Math.cos(180-tagAngle))
                .withVelocityY(swerveSupplier.supplyY().plus(MetersPerSecond.of(fieldCentricY))) // Drive left with negative X (left)
                .withRotationalRate(swerveSupplier.supplyRotationalRate()); // Drive counterclockwise with negative X (left)
    }

    public boolean isCoralTarget(int tagNumber){
        if(tagNumber == 17 || tagNumber == 6)
            return true;
        if(tagNumber == 18 || tagNumber == 7)
            return true;
        if(tagNumber == 19 || tagNumber == 8)
            return true;
        if(tagNumber == 20 || tagNumber == 9)
            return true;
        if(tagNumber == 21 || tagNumber == 10)
            return true;
        if(tagNumber == 22 || tagNumber == 11)
            return true;
        return false;
    }

    public void periodic(){

        if(limeLight.hasTarget() && isCoralTarget(limeLight.getTagID())){
            distance = limeLight.getDistanceToTag(0.017, 0.018, 0); // TODO: use not unreliable numbers
            tagAngle = limeLight.calculateAngleToTag();

            // Polar Coordinates to cartesian coordinates
            robotCentricX = distance * Math.cos(tagAngle - 15);
            robotCentricY = distance * Math.cos(tagAngle - 15);

            // rotate coordinates by angle of pigeon
            fieldCentricX = robotCentricX * Math.cos(robotAngle) - robotCentricY * Math.sin(robotAngle);
            fieldCentricY = robotCentricX * Math.cos(robotAngle) - robotCentricY * Math.cos(robotAngle);
        }
        else{
            fieldCentricX = 0;
            fieldCentricY = 0;
        }
    }

    private double calcTargetSpeed(double distance){
        return ((distance * distance) / 8 ) - 1; // we're using a parabola as an easing function
                                                 // the negative constant is so we reach 0 target speed before hitting the target
    }




}
