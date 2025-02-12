package frc.robot.subsystems.Prototype;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;

public interface PrototypeComponent {
    Command runBackward();
    Command runForward();
    Command stop();
    Command updateIds();

    void ClosedLoop(Consumer<PrototypeComponent> setup,Consumer<PrototypeComponent> periodic);
    void InjectControls(Consumer<PrototypeComponent>... config);
}