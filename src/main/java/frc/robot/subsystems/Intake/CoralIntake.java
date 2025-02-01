package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
    private TalonFX TopMotor;
    private TalonFX BottomMotor;
    
    private DutyCycleOut RunForwad= new DutyCycleOut(1);
    private DutyCycleOut RunBackward= new DutyCycleOut(-1);

    private NeutralOut Stop = new NeutralOut();


    public CoralIntake() {
        TopMotor = new TalonFX((int)SmartDashboard.getNumber("Intake/CoralIntakeTopMotor", 101));
        BottomMotor = new TalonFX((int)SmartDashboard.getNumber("Intake/CoralIntakeBottomMotor", 102));
    } 
    
    @Override
    public void periodic(){

    }

    public Command runForward(){
        return this.runOnce(()->{
            TopMotor.setControl(RunForwad);
            BottomMotor.setControl(RunBackward);
        });
    };

    public Command runBackward(){
        return this.runOnce(()->{
            TopMotor.setControl(RunBackward);
            BottomMotor.setControl(RunForwad);
        });
    };

    public Command stop(){
        return this.runOnce(()->{
            TopMotor.setControl(Stop);
            BottomMotor.setControl(Stop);
        });
    };

}