package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.hci.swerve.SwerveSupplier;
import frc.robot.subsystems.RotationController;
import frc.robot.subsystems.vision.Pipeline;
import frc.robot.subsystems.vision.VisionTypes;
import frc.robot.util.BSLogger;

import java.util.Optional;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

public class TrackNeuralTargetCommand extends Command {
    private final SwerveSupplier swerveSupplier;
    private SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric();
    private final RotationController pid = new RotationController(0.016, 0, 0);
    private Angle targetAngle = Degrees.of(-12);


    public TrackNeuralTargetCommand(SwerveSupplier swerveSupplier) {
        this.swerveSupplier = swerveSupplier;
        addRequirements(Subsystems.swerveSubsystem);
    }

    @Override
    public void initialize() {
        Subsystems.visionSubsystem.selectPipeline(Pipeline.NeuralDetector);
        pid.reset();
        pid.setTolerance(0.5);
    }

    @Override
    public void execute() {
        Optional<VisionTypes.TargetInfo> target = Subsystems.visionSubsystem.getTargetInfo();
        AngularVelocity rotation = swerveSupplier.supplyRotationalRate();
        if (target.isPresent() && target.get().hasTarget()) {
            double tx = target.get().xOffset();
            double error = pid.calculate(tx, targetAngle.in(Degrees));
            rotation = Constants.MaxAngularRate.times(error);
        }

        BSLogger.log("TrackNeural", "Rotation: " + rotation.in(DegreesPerSecond));

        Subsystems.swerveSubsystem.setControl(
                robotCentric
                        .withVelocityX(swerveSupplier.supplyX())
                        .withVelocityY(swerveSupplier.supplyY())
                        .withRotationalRate(rotation)
        );
    }

    @Override
    public void end(boolean interrupted) {
        Subsystems.visionSubsystem.selectPipeline(Pipeline.April);
    }
}
