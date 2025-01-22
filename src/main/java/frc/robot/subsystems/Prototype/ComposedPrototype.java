package frc.robot.subsystems.Prototype;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Consumer;
import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ComposedPrototype implements PrototypeGeneric{
    PrototypeGeneric[] prototypeComponents;
    

    public ComposedPrototype(PrototypeGeneric... gProtoypes){
        prototypeComponents = gProtoypes;
    }

    public Command runForward(){
        return new SequentialCommandGroup(
            commandComponents(PrototypeGeneric::runForward)
        );
    }

    public Command runBackward(){
        return new SequentialCommandGroup(
            commandComponents(PrototypeGeneric::runBackward)
        );
    }

    public Command stop(){
        return new SequentialCommandGroup(
            commandComponents(PrototypeGeneric::stop)
        );
    }

    public Command updateIds(){
        return new SequentialCommandGroup(
            commandComponents(PrototypeGeneric::updateIds)
        );
    }

    public PrototypeGeneric getComponent(int componentIndex){
        return prototypeComponents[componentIndex];
    }

    public PrototypeGeneric[] getComponentList(){
        return prototypeComponents;
    }

    public ArrayList<PrototypeGenericMotor> getMotorList(){
        ArrayList<PrototypeGenericMotor> motors = new ArrayList<>(0);
        for(PrototypeGeneric component : prototypeComponents){
            if(component instanceof PrototypeGenericMotor)
                motors.add((PrototypeGenericMotor)component);
        }
        return motors;
    }
    
    private Command[] commandComponents(Function<PrototypeGeneric, Command> extractor){
        return Arrays.stream(prototypeComponents)
                .map(extractor)
                .toArray(Command[]::new);
    }

    
    
    
}

