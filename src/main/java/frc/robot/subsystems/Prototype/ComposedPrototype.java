package frc.robot.subsystems.Prototype;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Consumer;
import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ComposedPrototype implements PrototypeComponent{
    PrototypeComponent[] prototypeComponents;
    // need do: 

    public ComposedPrototype(PrototypeComponent... gProtoypes){
        prototypeComponents = gProtoypes;
    }

    public Command runForward(){
        return new SequentialCommandGroup(
            commandComponents(PrototypeComponent::runForward)
        );
    }

    public Command runBackward(){
        return new SequentialCommandGroup(
            commandComponents(PrototypeComponent::runBackward)
        );
    }

    public Command stop(){
        return new SequentialCommandGroup(
            commandComponents(PrototypeComponent::stop)
        );
    }

    public Command updateIds(){
        return new SequentialCommandGroup(
            commandComponents(PrototypeComponent::updateIds)
        );
    }

    public PrototypeComponent getComponent(int componentIndex){
        return prototypeComponents[componentIndex];
    }

    public PrototypeComponent[] getComponentList(){
        return prototypeComponents;
    }

    public ArrayList<ComponentMotor> getMotorList(){
        ArrayList<ComponentMotor> motors = new ArrayList<>(0);
        for(PrototypeComponent component : prototypeComponents){
            if(component instanceof ComponentMotor)
                motors.add((ComponentMotor)component);
        }
        return motors;
    }
    
    private Command[] commandComponents(Function<PrototypeComponent, Command> extractor){
        return Arrays.stream(prototypeComponents)
                .map(extractor)
                .toArray(Command[]::new);
    }
    
    public void InjectControls(Consumer<PrototypeComponent>... config){
        
    }

    public void ClosedLoop(Consumer<PrototypeComponent> setup,Consumer<PrototypeComponent> periodic){
        
    }
    
}

