package frc.robot.hci.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems;
import frc.robot.subsystems.Prototype.ComponentPreconfig;

public class AustinGearboxPrototypeControls extends ControlBinding {

    public AustinGearboxPrototypeControls(Joystick driveStick, Joystick steerStick, CommandXboxController joystick) {
        super(driveStick, steerStick, joystick);
    }

    @Override
    public void bindControls() {
        Subsystems.austinGearPrototype.InjectControls(ComponentPreconfig.ABXYpreconf);
    }
}
