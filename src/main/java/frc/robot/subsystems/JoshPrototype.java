package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class JoshPrototype {
  private TalonFX frontMotor;
  private TalonFX backMotor;   
 private final DutyCycleOut stop_request = new DutyCycleOut(0);
 private final DutyCycleOut injest_request = new DutyCycleOut(0);
 private final DutyCycleOut eject_request = new DutyCycleOut(0);



public void moveMotor(){
frontMotor = new TalonFX(50);
backMotor = new TalonFX(51);

}

public Command stop() {
   
    return Commands.runOnce(() -> {
      this.frontMotor.setControl(stop_request.withOutput(0.0));
      this.backMotor.setControl(stop_request.withOutput(0.0));
    });

}

public Command ingest() {
    return Commands.runOnce(() -> {
    this.frontMotor.setControl(injest_request.withOutput(0.05));
    this.backMotor.setControl(injest_request.withOutput(-0.05));
});
}


public Command eject() {
    return Commands.runOnce(() -> {
    this.frontMotor.setControl(eject_request.withOutput(-0.1));
    this.backMotor.setControl(eject_request.withOutput(0.1));
    }); 
}


}
