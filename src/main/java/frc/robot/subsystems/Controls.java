package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Controls {
    public static CommandXboxController joystick = new CommandXboxController(2);
    public static Joystick left = new Joystick(0);
    public static Joystick right = new Joystick(1);

    public Controls(CommandXboxController gamePad, Joystick L, Joystick R){
        joystick = gamePad;
        left = L;
        right = R;
    }


}
