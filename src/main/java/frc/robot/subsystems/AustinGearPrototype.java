package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class AustinGearPrototype implements Lifecycle, Subsystem {
    private TalonFX motor;

    private final NeutralOut stop = new NeutralOut();
    private final DutyCycleOut forward = new DutyCycleOut(0);
    private final DutyCycleOut backward = new DutyCycleOut(0);

    public AustinGearPrototype(){
        motor = new TalonFX(50);

        motor.getConfigurator().apply((new TalonFXConfiguration()));
        motor.setNeutralMode(NeutralModeValue.Brake);

        SmartDashboard.setDefaultNumber("austinGearPrototype/runForward", -0.01);
        SmartDashboard.setDefaultNumber("austinGearPrototype/runBackward", -0.01);
    }

    public Command runForward(){
        return this.runOnce( () -> {
        motor.setControl(forward.withOutput(SmartDashboard.getNumber("austinGearPrototype/runForward", 0)));
        }
        );
    }

    public Command runBackward(){
        return this.runOnce(()->{
        motor.setControl(backward.withOutput(SmartDashboard.getNumber("austinGearPrototype/runBackward", 0)));
        });
    }

    public Command stopMotor(){
        return this.runOnce(() -> {
            motor.setControl(stop);
        });
    }
}
