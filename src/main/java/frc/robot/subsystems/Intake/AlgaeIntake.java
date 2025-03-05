package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.Lifecycle;
import frc.robot.util.MotorVoltageFilter;

public class AlgaeIntake extends SubsystemBase implements Lifecycle {
    private final TalonFX algaeIntakeMotor = new TalonFX(Robot.robotConfig.getCanID("algaeIntakeMotor"));
    private final NeutralOut brake = new NeutralOut();
    private final DutyCycleOut intakeDutyCycleOut = new DutyCycleOut(1);
//    private final MotorVoltageFilter motorVoltageFilter = new MotorVoltageFilter(5, algaeIntakeMotor); //TODO: 5 isnt a real number here


    private double forwardSpeed = 0.65;
    private double backwardSpeed = -0.3;
    private double holdSpeed = 0.15;

    public AlgaeIntake() {

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration intakeConfiguration = new TalonFXConfiguration()
                .withMotorOutput(motorOutputConfigs);
        algaeIntakeMotor.getConfigurator().apply(intakeConfiguration);

        this.setDefaultCommand(new DefaultHoldCommand());
    }

    public void setForwardSpeed(double speed) {
        this.forwardSpeed = speed;
    }

    public void setBackwardSpeed(double speed) {
        this.backwardSpeed = speed;
    }

    public void setHoldSpeed(double speed) {
        this.holdSpeed = speed;
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("AlgaeIntake");
        builder.addDoubleProperty("forwardSpeed", () -> forwardSpeed, this::setForwardSpeed);
        builder.addDoubleProperty("backwardSpeed", () -> backwardSpeed, this::setBackwardSpeed);
        builder.addDoubleProperty("holdSpeed", () -> holdSpeed, this::setHoldSpeed);
    }

    public Command intakeCommand() {
        return this.run(() -> algaeIntakeMotor.setControl(intakeDutyCycleOut.withOutput(forwardSpeed))).withName("Algae Intake");
    }

    public Command ejectCommand() {
        return this.run(() -> algaeIntakeMotor.setControl(intakeDutyCycleOut.withOutput(backwardSpeed))).withName("Algae Eject");

    }

    public Command holdAlgaeCommand() {
        return this.run(() -> algaeIntakeMotor.setControl(intakeDutyCycleOut.withOutput(holdSpeed))).withName("Algae Hold");
    }

    public Command stopCommand() {
        return this.run(() -> algaeIntakeMotor.setControl(brake)).withName("Algae Stop");
    }

    class DefaultHoldCommand extends Command {
        DefaultHoldCommand() {
            addRequirements(AlgaeIntake.this);
            setName("Algae Hold");
        }

        @Override
        public void initialize() {
            algaeIntakeMotor.setControl(brake);
//            algaeIntakeMotor.setControl(intakeDutyCycleOut.withOutput(0.25));
        }
    }


}
