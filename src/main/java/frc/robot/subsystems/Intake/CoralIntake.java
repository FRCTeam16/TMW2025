package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

public class CoralIntake extends SubsystemBase {
    private TalonFX TopMotor;
    private TalonFX BottomMotor;
    
    private DutyCycleOut RunForwad = new DutyCycleOut(1);
    private DutyCycleOut RunBackward = new DutyCycleOut(-1);

    private LaserCan laser1 = new LaserCan(111);
    private LaserCan laser2 = new LaserCan(112);

    private NeutralOut Stop = new NeutralOut();

    //TODO: GET REAL NUMS
    int laser1SenseDistance = 3;
    int laser2SenseDistance = 3;
    double intakeHighSpeed = 0.7;
    double intakeLowSpeed = 0.2;
    //TODO: GET REAL NUMS


    public CoralIntake() {
        TopMotor = new TalonFX((int)SmartDashboard.getNumber("Intake/CoralIntakeTopMotor", 101));
        BottomMotor = new TalonFX((int)SmartDashboard.getNumber("Intake/CoralIntakeBottomMotor", 102));
    } 
    
    @Override
    public void periodic(){
        RunForwad = new DutyCycleOut(SmartDashboard.getNumber("Intake/CoralIntakeSpeed", 1));
        RunForwad = new DutyCycleOut(SmartDashboard.getNumber("Intake/CoralIntakeSpeed", -1));
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

    public Command intakeCoral(){
        return new Command() {
            int step = 1;
            boolean shooting = false;
            @Override
            public void execute(){
                
                //default action when intake: runForward(fast)
                if(step == 1){
                    SmartDashboard.putNumber("Intake/CoralIntakeSpeed", intakeHighSpeed); 
                    //if first laser sees coral while default action: change action to action 2
                    if( laser1.getMeasurement().distance_mm > laser1SenseDistance){ 
                        step = 2;
                    }
                }

                //secound action when intake: runForward(slow)
                if(step == 2){
                    SmartDashboard.putNumber("Intake/CoralIntakeSpeed", intakeLowSpeed); 
                    //if secound laser sees coral while secound action: change action to action 3
                    if(laser2.getMeasurement().distance_mm > laser2SenseDistance){
                        step = 3;
                    }
                }

                //thrid action when intake: stop
                if(step == 3){
                    //if shooting == true {runForwards(fast)}
                    if(shooting){
                        SmartDashboard.putNumber("Intake/CoralIntakeSpeed", intakeHighSpeed);
                    }else{
                        //defualt action when we're doing nothing: stop
                        stop();
                    }
                    //if we've stopped shooting and secound laser sees nothing: go back to default action
                    if(!shooting && laser2.getMeasurement().distance_mm <= laser2SenseDistance){
                        step = 1;
                    }
                }
            }
        };
    }

}