package frc.robot.hci;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.util.GameInfo;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class JoystickSwerveSupplier implements SwerveSupplier {
    private final Joystick steerStick;
    private final Joystick driveStick;
    private final CommandXboxController controller;
    private final boolean isRedAlliance;   // TODO Centralized world state lookup?

    public JoystickSwerveSupplier(Joystick driveStick, Joystick steerStick, CommandXboxController controller) {
        this.steerStick = steerStick;
        this.driveStick = driveStick;
        this.controller = controller;
        this.isRedAlliance = GameInfo.isRedAlliance();
    }

    @Override
    public LinearVelocity supplyX() {
        double base = driveStick.getY() * (isRedAlliance ? -1 : 1);
        return MetersPerSecond.of(base).times(Constants.MaxSpeed.in(MetersPerSecond));
    }

    @Override
    public LinearVelocity supplyY() {
        double base = driveStick.getX() * (isRedAlliance ? -1 : 1);
        return MetersPerSecond.of(base).times(Constants.MaxSpeed.in(MetersPerSecond));
    }

    @Override
    public AngularVelocity supplyRotationalRate() {
        double base = steerStick.getX() * (isRedAlliance ? -1 : 1);
        return RadiansPerSecond.of(base).times(Constants.MaxAngularRate.in(RadiansPerSecond));
    }
}