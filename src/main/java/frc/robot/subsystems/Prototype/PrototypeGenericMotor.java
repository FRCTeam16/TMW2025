package frc.robot.subsystems.Prototype;

import java.util.function.Consumer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Lifecycle;

public class PrototypeGenericMotor implements Lifecycle, Subsystem, PrototypeGeneric {
    public enum direction{
        inverse(-1),
        corresponding(1);

        public final int value;

        private direction(int dir){
            this.value = dir;
        }
    }

    private TalonFX motor;

    private final NeutralOut stop = new NeutralOut();
    private final DutyCycleOut forward = new DutyCycleOut(0.3);
    private final DutyCycleOut backward = new DutyCycleOut(-0.3);
    private String ElasticName;

    // Overloaded constructor for default configs
    public PrototypeGenericMotor(String ElasticName, int defaultId){
        this(ElasticName, defaultId, (temp)->{});
    }

    public PrototypeGenericMotor(String ElasticName, int defaultId, Consumer<PrototypeGenericMotor> config){
        this.ElasticName = ElasticName;
        SmartDashboard.setDefaultNumber(ElasticName + "/motorID", defaultId);
        motor = new TalonFX((int)SmartDashboard.getNumber(ElasticName + "/motorID", defaultId));
        
        motor.getConfigurator().apply((new TalonFXConfiguration()));
        motor.setNeutralMode(NeutralModeValue.Brake);

        SmartDashboard.setDefaultNumber(ElasticName + "/runForward", -0.3);
        SmartDashboard.setDefaultNumber(ElasticName + "/runBackward", 0.3);
        
        config.accept(this);
    }

    
    public Command runForward(){
        return this.runOnce(() -> {
        motor.setControl(forward.withOutput(SmartDashboard.getNumber(ElasticName+"/runForward", 0)));
        System.out.println(SmartDashboard.getNumber(ElasticName+"/runForward", 0));
        });
    }

    public Command runBackward(){
        return this.runOnce(() -> {
        motor.setControl(backward.withOutput(SmartDashboard.getNumber(ElasticName + "/runBackward", 0)));
        System.out.println(SmartDashboard.getNumber(ElasticName+"/runBackward", 0));
        });
    }

    public Command stop(){
        return this.runOnce(() -> {
            motor.setControl(stop);
        });
    }

    public Command updateIds(){
        return this.runOnce(() -> {
            motor = new TalonFX((int)SmartDashboard.getNumber(ElasticName + "/motorID", 51));
        });
    }

    public Command setDirection(PrototypeGenericMotor.direction dir){ // generally usedfull for config lambdas
        return this.runOnce(() -> {
            SmartDashboard.setDefaultNumber(ElasticName + "/runForward", SmartDashboard.getNumber(ElasticName + "/runForward", 0) * dir.value);
            SmartDashboard.setDefaultNumber(ElasticName + "/runBackward", SmartDashboard.getNumber(ElasticName + "/runBackward", 0) * -dir.value);
        });
    }
}
