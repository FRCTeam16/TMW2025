package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.RobotConfig;
import frc.robot.subsystems.Lifecycle;
import frc.robot.util.BSLogger;

public class FunnelSubsystem extends SubsystemBase implements Lifecycle {
    private final TalonFX pivotMotor = new TalonFX(RobotConfig.getInstance().getCanID("funnelPivotMotor"));
    private final TalonFX conveyorMotor = new TalonFX(RobotConfig.getInstance().getCanID("funnelConveyorMotor"));

    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(0);
    private final StaticBrake brake = new StaticBrake();

    private double pivotSetpoint = 0.0;
    private double conveyorSpeed = 0.15;

    public FunnelSubsystem() {
        TalonFXConfiguration conveyorMotorConfigs = new TalonFXConfiguration();
        conveyorMotor.getConfigurator().apply(conveyorMotorConfigs);


        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();
        Slot0Configs slot0Configs = new Slot0Configs()
                .withKP(0.6);
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(10)
                .withMotionMagicAcceleration(20);
        pivotMotorConfig
                .withMotorOutput(motorOutputConfigs)
                .withSlot0(slot0Configs)
                .withMotionMagic(motionMagicConfigs);

        pivotMotor.getConfigurator().apply(pivotMotorConfig);
        this.setDefaultCommand(this.defaultCommand());
    }

    @Override
    public void teleopInit() {
        this.pivotSetpoint = pivotMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("FunnelSubsystem");
        builder.addDoubleProperty("conveyorSpeed", () -> conveyorSpeed, (v) -> conveyorSpeed = v);
        builder.addDoubleProperty("currentPosition", () -> pivotMotor.getPosition().getValueAsDouble(), null);
        builder.addDoubleProperty("rotationSetpoint", () -> pivotSetpoint, (v) -> pivotSetpoint = v);
    }

    void holdPosition() {
        pivotMotor.setControl(motionMagic.withPosition(pivotSetpoint));
    }

    void startConveyor() {
        conveyorMotor.setControl(dutyCycleOut.withOutput(conveyorSpeed));
    }

    void stopConveyor() {
        conveyorMotor.setControl(dutyCycleOut.withOutput(0));
    }

    private Command defaultCommand() {
        return this.run(() -> {
            this.holdPosition();
            this.stopConveyor();
        }).withName("Default Funnel Hold");
    }

    public Command holdPositionCommand() {
        return Commands.run(this::holdPosition).withName("Hold Funnel Position");
    }

    public Command movePivotToPosition(double position) {
        this.pivotSetpoint = position;
        return holdPositionCommand();
    }

    public Command startConveyorCommand() {
        return this.run(this::startConveyor).withName("Start Conveyor");
    }

    public Command stopConveyorCommand() {
        return this.run(this::stopConveyor).withName("Stop Conveyor");
    }

}
