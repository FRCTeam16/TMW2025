package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PrototypeGenericMotor implements Lifecycle, Subsystem {
    private TalonFX motor;

    private final NeutralOut stop = new NeutralOut();
    private final DutyCycleOut forward = new DutyCycleOut(0);
    private final DutyCycleOut backward = new DutyCycleOut(0);
    private String ElasticName;

    public PrototypeGenericMotor(String ElasticName, int defaultId){
        this.ElasticName = ElasticName;
        motor = new TalonFX((int)SmartDashboard.getNumber(ElasticName + "/motorID", defaultId));
        

        motor.getConfigurator().apply((new TalonFXConfiguration()));
        motor.setNeutralMode(NeutralModeValue.Brake);

        SmartDashboard.setDefaultNumber(ElasticName + "/runForward", -0.3);
        SmartDashboard.setDefaultNumber(ElasticName + "/runBackward", 0.3);
    }

    public Command runForward(){
        return this.runOnce( () -> {
        motor.setControl(forward.withOutput(SmartDashboard.getNumber(ElasticName+"/runForward", 0)));
        }
        );
    }

    public Command runBackward(){
        return this.runOnce(()->{
        motor.setControl(backward.withOutput(SmartDashboard.getNumber(ElasticName + "PrototypeGeneric/runBackward", 0)));
        });
    }

    public Command stopMotor(){
        return this.runOnce(() -> {
            motor.setControl(stop);
        });
    }

    public Command updateMotorIds(){
        return this.runOnce(() -> {
            motor = new TalonFX((int)SmartDashboard.getNumber(ElasticName + "/motorID", 51));
        });
    }
}
