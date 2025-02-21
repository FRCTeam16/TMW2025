package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.RobotConfig;
import frc.robot.subsystems.Lifecycle;

public class FunnelSubsystem extends SubsystemBase implements Lifecycle {
    private final TalonFX pivotMotor = new TalonFX(RobotConfig.getInstance().getCanID("funnelPivotMotor"));
    private final TalonFX conveyorMotor = new TalonFX(RobotConfig.getInstance().getCanID("funnelConveyorMotor"));

    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final StaticBrake brake = new StaticBrake();

    private double conveyorSpeed = 0.5;

    public FunnelSubsystem() {
        TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();
        pivotMotor.getConfigurator().apply(pivotMotorConfig);
        this.setDefaultCommand(this.defaultCommand());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("FunnelSubsystem");
        builder.addDoubleProperty("conveyorSpeed", () -> conveyorSpeed, (v) -> conveyorSpeed = v);
    }

    void holdPosition() {
        pivotMotor.setControl(brake);
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

    public Command startConveyorCommand() {
        return this.run(this::startConveyor).withName("Start Conveyor");
    }

    public Command stopConveyorCommand() {
        return this.run(this::stopConveyor).withName("Stop Conveyor");
    }

}
