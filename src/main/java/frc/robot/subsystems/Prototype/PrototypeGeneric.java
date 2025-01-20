package frc.robot.subsystems.Prototype;

import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj2.command.Command;

public interface PrototypeGeneric {
    Command runBackward();
    Command runForward();
    Command stop();
    Command updateIds();

    static <T extends PrototypeGeneric> List<T> filterByType(List<PrototypeGeneric> prototypes, Class<T> type) {
    return prototypes.stream()
            .filter(type::isInstance)  // Filter by the specific type
            .map(type::cast)           // Cast to the target type
            .collect(Collectors.toList());
    }
}