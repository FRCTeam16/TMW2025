package frc.robot.subsystems.Prototype;

import edu.wpi.first.wpilibj2.command.Command;

public interface PrototypeGeneric {
    Command runBackward();
    Command runForward();
    Command stop();
    Command updateIds();
}
