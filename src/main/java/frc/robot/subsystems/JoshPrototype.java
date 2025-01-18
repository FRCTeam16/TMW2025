package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class JoshPrototype implements Lifecycle, Subsystem {
  private TalonFX frontMotor;
  private TalonFX backMotor;
  // private final DutyCycleOut stop_request = new DutyCycleOut(0);
  private final NeutralOut stop_request = new NeutralOut();
  private final DutyCycleOut injest_request = new DutyCycleOut(0);
  private final DutyCycleOut eject_request = new DutyCycleOut(0);

  public JoshPrototype() {
    frontMotor = new TalonFX(50);
    backMotor = new TalonFX(51);

    frontMotor.getConfigurator().apply(new TalonFXConfiguration());
    backMotor.getConfigurator().apply(new TalonFXConfiguration());

    frontMotor.setNeutralMode(NeutralModeValue.Brake);
    backMotor.setNeutralMode(NeutralModeValue.Brake);

    SmartDashboard.setDefaultNumber("JoshPrototype/ejectFront", -0.1);
    SmartDashboard.setDefaultNumber("JoshPrototype/ejectBack", 0.1);
    SmartDashboard.setDefaultNumber("JoshPrototype/ingestFront", 0.05);
    SmartDashboard.setDefaultNumber("JoshPrototype/ingestBack", -0.05);
  }

  public Command stop() {
    return this.runOnce(() -> {
      // this.frontMotor.setControl(stop_request.withOutput(0.0));
      // this.backMotor.setControl(stop_request.withOutput(0.0));
      this.frontMotor.setControl(stop_request);
      this.backMotor.setControl(stop_request);
    });

  }

  public Command ingest() {
    return this.runOnce(() -> {
      double ingestFront = SmartDashboard.getNumber("JoshPrototype/ingestFront", 0);
      double ingestBack = SmartDashboard.getNumber("JoshPrototype/ingestBack", 0);
      this.frontMotor.setControl(injest_request.withOutput(ingestFront));
      this.backMotor.setControl(injest_request.withOutput(ingestBack));
    });
  }

  public Command eject() {
    return this.runOnce(this::doEject);
  }

  private void doEject() {
    double ejectFront = SmartDashboard.getNumber("JoshPrototype/ejectFront", 0);
    double ejectBack = SmartDashboard.getNumber("JoshPrototype/ejectBack", 0);
    this.frontMotor.setControl(eject_request.withOutput(ejectFront));
    this.backMotor.setControl(eject_request.withOutput(ejectBack));
  }

}
