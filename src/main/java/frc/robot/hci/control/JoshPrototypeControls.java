package frc.robot.hci.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems;

public class JoshPrototypeControls extends ControlBinding {

    public JoshPrototypeControls(Joystick driveStick, Joystick steerStick, CommandXboxController joystick) {
        super(driveStick, steerStick, joystick);
    }

    @Override
    public void bindControls() {
        joystick.b().onTrue(Subsystems.joshPrototype.stop());
        joystick.y().onTrue(Subsystems.joshPrototype.eject()).onFalse(Subsystems.joshPrototype.stop());
        joystick.a().onTrue(Subsystems.joshPrototype.ingest()).onFalse(Subsystems.joshPrototype.stop());
    }

}
