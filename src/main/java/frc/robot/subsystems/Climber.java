package frc.robot.subsystems;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

public class Climber extends SubsystemBase {

    private TalonFX climberHand = new TalonFX(1); 
    private TalonFX climberPivot = new TalonFX(2); 

    private DutyCycleOut runOpenLoop = new DutyCycleOut(0);
    private DutyCycleOut runClosedLoop = new DutyCycleOut(0);

    private double openLoopMotorOutput = 0.5; 
    private double closedLoopMotorOutput = 0;
    private double currentSetpoint = 0;
    private boolean isOpenloop = true;

    private PIDController climberPID = new PIDController(0.1, 0, 0); // Example PID values

    @Override
    public void periodic() {
        closedLoopMotorOutput = climberPID.calculate(currentSetpoint);
        runClosedLoop = new DutyCycleOut(closedLoopMotorOutput);

        if (!isOpenloop) {
            climberHand.setControl(runClosedLoop);
            climberPivot.setControl(runClosedLoop);
        }
    }

    public Command openLoopPivot() {
        return this.runOnce(() -> {
            runOpenLoop = new DutyCycleOut(openLoopMotorOutput);
            climberPivot.setControl(runOpenLoop);
        });
    }

    public Command openLoopHand() {
        return this.runOnce(() -> {
            runOpenLoop = new DutyCycleOut(openLoopMotorOutput);
            climberHand.setControl(runOpenLoop);
        });
    }

}
