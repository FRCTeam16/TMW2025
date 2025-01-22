package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.vision.LimelightHelpers;

import static edu.wpi.first.units.Units.MetersPerSecond;

public class AlignmentTest extends Command {

    SwerveRequest.RobotCentric alignDrive = new SwerveRequest.RobotCentric();

    public AlignmentTest() {
        this.addRequirements(Subsystems.swerveSubsystem);
    }

    @Override
    public void execute() {
        // Handle right input
        var limelightName = "limelight-right";
        var visionX = LimelightHelpers.getTX(limelightName);
        var targetX = 3.0;
        var errorX = MathUtil.clamp(visionX - targetX, -4, 4);
        var robotSpeed = 0.5;

        var rawSpeed = robotSpeed * errorX;
        LinearVelocity yDriveSpeed = MetersPerSecond.of(MathUtil.clamp(rawSpeed, -0.5, 0.5));

        Subsystems.swerveSubsystem.setControl(
                alignDrive.withVelocityX(0)
                        .withRotationalRate(0)
                        .withVelocityY(yDriveSpeed.times(-1)));
    }
}
