package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Degrees;

public class ZeroYawCommand extends InstantCommand {

    private final Angle angle;

    public ZeroYawCommand() {
        this(Degrees.of(0.0));
    }

    public ZeroYawCommand(Angle offset) {
        this.angle = offset;
    }

    @Override
    public void initialize() {
        Subsystems.swerveSubsystem.getPigeon2().setYaw(angle.in(Degrees));
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
