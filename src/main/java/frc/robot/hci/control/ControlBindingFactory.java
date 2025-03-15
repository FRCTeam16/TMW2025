package frc.robot.hci.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControlBindingFactory {

    public static ControlBinding createControlBinding(JoystickMode mode, Joystick driveStick, Joystick steerStick, CommandXboxController joystick) {
        Class<? extends ControlBinding> bindingClass = switch (mode) {
            case CompBot -> CompBotControls.class;
            case CompBotDev -> CompBotDevControls.class;
            case Scrimmage -> ScrimmageControls.class;
            case JoshPrototype -> JoshPrototypeControls.class;
            case AustinGearboxPrototype -> AustinGearboxPrototypeControls.class;
            case AlignmentTest -> AlignmentTestControls.class;
            case ElevatorProto -> ElevatorProtoControls.class;
            case climberProto -> ClimberProtoControls.class;
            case AlgaeProto -> AlgaeProtoControls.class;
            case PathTesting -> PathTestingControls.class;
            case SysId -> SysIdControls.class;
            case CoralTesting -> CoralTestingControls.class;
            case Bayou -> BayouControls.class;
            case none -> DefaultControls.class;
        };

        try {
            ControlBinding binding = bindingClass.getConstructor(Joystick.class, Joystick.class, CommandXboxController.class)
                                                 .newInstance(driveStick, steerStick, joystick);
            binding.bindControls();
            return binding;
        } catch (Exception e) {
            throw new RuntimeException("Failed to create control binding", e);
        }
    }

    public enum JoystickMode {
        CompBot,
        CompBotDev,
        Scrimmage,
        JoshPrototype,
        AustinGearboxPrototype,
        AlignmentTest,
        ElevatorProto,
        climberProto,
        AlgaeProto,
        PathTesting,
        SysId,
        CoralTesting,
        Bayou,
        none
    }
}