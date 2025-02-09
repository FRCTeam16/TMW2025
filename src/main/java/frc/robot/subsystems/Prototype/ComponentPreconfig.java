package frc.robot.subsystems.Prototype;

import java.util.function.BiConsumer;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Controls;

public class ComponentPreconfig {

    static public final Consumer<PrototypeComponent> ABXYpreconf = (proto) -> {
                Controls.joystick.b().onTrue(proto.stop());
                Controls.joystick.a().onTrue(proto.runForward()).onFalse(proto.stop());
                Controls.joystick.y().onTrue(proto.runBackward()).onFalse(proto.stop());
                Controls.joystick.x().onTrue(proto.updateIds()); //IDs are ran through elastic
    };

    static public final BiConsumer<PrototypeComponent, String> PreElasticConfig = (proto, elasticName) -> {
        selectedButton(elasticName).onTrue(getCommandOfString(elasticName, proto));
    };

    private static Trigger selectedButton(String ComponentName) {
        String selectedButton = SmartDashboard.getString(ComponentName + "/button", null);

        return switch (selectedButton) {
            case "A"  -> Controls.joystick.a();
            case "B"  -> Controls.joystick.b();
            case "X"  -> Controls.joystick.x();
            case "Y"  -> Controls.joystick.y();
            case "LB" -> Controls.joystick.leftBumper();
            case "RB" -> Controls.joystick.rightBumper();
            default   -> null;
        };
    }

    private static Command getCommandOfString(String ComponentName, PrototypeComponent proto){
        String in = SmartDashboard.getString(ComponentName + "/Command", "");
        
        return switch (in) {
        case "runForward"  -> proto.runForward();
        case "runBackward" -> proto.runBackward();
        case "stop"        -> proto.stop();
        case "updateIds"   -> proto.updateIds();
        default            -> new InstantCommand(); // No-op command as a fallback
    };
    }
}
