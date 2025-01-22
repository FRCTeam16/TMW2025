package frc.robot.subsystems.Prototype;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj2.command.Command;

public interface PrototypeGeneric {
    Command runBackward();
    Command runForward();
    Command stop();
    Command updateIds();

}