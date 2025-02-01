package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AgleaIntake extends SubsystemBase {
    private TalonFX TopMotor;
    private TalonFX BottomMotor;

    private NeutralOut brake = new NeutralOut();
    
    private DutyCycleOut RunForward = new DutyCycleOut(1);
    private DutyCycleOut RunBackward = new DutyCycleOut(-1);

    public AgleaIntake() {
        TopMotor = new TalonFX((int)SmartDashboard.getNumber("Intake/AlgeaIntakeTopMotor", 101));
        BottomMotor = new TalonFX((int)SmartDashboard.getNumber("Intake/AlgeaIntakeBottomMotor", 102));
    }

    @Override
    public void periodic(){

    }

    public Command runForward(){
        return this.runOnce(() -> {
            TopMotor.setControl(RunForward);
            BottomMotor.setControl(RunBackward);
        });
    }

    public Command runBackward(){
        return this.runOnce(() -> {
            TopMotor.setControl(RunBackward);
            BottomMotor.setControl(RunForward);
        });
    }

    public Command stop(){
        return this.runOnce(() -> {
            TopMotor.setControl(brake);
            BottomMotor.setControl(brake);
        });
    }

}
