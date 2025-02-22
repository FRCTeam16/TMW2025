package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems;
import frc.robot.hci.swerve.SwerveSupplier;

public class DriveRobotCentricCommand extends Command {
    private SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric();
    public DriveRobotCentricCommand() {
        addRequirements(Subsystems.swerveSubsystem);
    }

    @Override
    public void initialize() {
        SwerveSupplier supplier = RobotContainer.getInstance().getSwerveSupplier();
        Subsystems.swerveSubsystem.setControl(
                robotCentricRequest
                        .withVelocityX(supplier.supplyX())
                        .withVelocityY(supplier.supplyY())
                        .withRotationalRate(supplier.supplyRotationalRate())
        );
    }

    @Override
    public void execute() {
        super.execute();
    }
}
