package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.ctre.phoenix6.signals.S2FloatStateValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.subsystems.Lifecycle;

public class CoralIntake extends SubsystemBase implements Lifecycle {
    public static final double COMMAND_TIMEOUT_SECONDS = 5.0;
    private final TalonFX topMotor = new TalonFX(Robot.robotConfig.getCanID("coralIntakeLeftMotor"));
    private final TalonFX bottomMotor = new TalonFX(Robot.robotConfig.getCanID("coralIntakeRightMotor"));
    private final DutyCycleOut dutyCycleOutTop = new DutyCycleOut(1);
    private final DutyCycleOut dutyCycleOutBottom = new DutyCycleOut(1);

    private final CANdi candi = new CANdi(Robot.robotConfig.getCanID("coralIntakeCandi"));

    private final Debouncer topSensorFilter = new Debouncer(0.005);
    private final Debouncer bottomSensorFilter = new Debouncer(0.005);

    private final StaticBrake stop = new StaticBrake();


    double intakeHighSpeedLeft = 0.5;
    double intakeHighSpeedRight = -0.5;
    double intakeLowSpeed = 0.2;
    double ejectSpeed = -0.3;
    private boolean topSensorDetected;
    private boolean bottomSensorDetected;


    public CoralIntake() {
        CANdiConfiguration candiConfig = new CANdiConfiguration()
                .withDigitalInputs(new DigitalInputsConfigs()
                        .withS1CloseState(S1CloseStateValue.CloseWhenHigh)
                        .withS1FloatState(S1FloatStateValue.PullLow)
                        .withS2CloseState(S2CloseStateValue.CloseWhenHigh)
                        .withS2FloatState(S2FloatStateValue.PullLow));
        this.candi.getConfigurator().apply(candiConfig);
        this.setDefaultCommand(this.stopCommand());

    }

    @Override
    public void robotInit() {
        Subsystems.asyncManager.register("CoralPeriodicDetect", this::updateDetectorState);
    }

    private void updateDetectorState() {
        topSensorDetected = topSensorFilter.calculate(candi.getS1Closed().getValue());
        bottomSensorDetected = bottomSensorFilter.calculate(candi.getS2Closed().getValue());
    }

    @Override
    public void periodic() {
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("CoralIntake");
        builder.addDoubleProperty("intakeHighSpeedLeft", () -> intakeHighSpeedLeft, (v) -> intakeHighSpeedLeft = v);
        builder.addDoubleProperty("intakeHighSpeedRight", () -> intakeHighSpeedRight, (v) -> intakeHighSpeedRight = v);

        builder.addDoubleProperty("intakeLowSpeed", () -> intakeLowSpeed, (v) -> intakeLowSpeed = v);
        builder.addDoubleProperty("ejectSpeed", () -> ejectSpeed, (v) -> ejectSpeed = v);
        builder.addBooleanProperty("coralDetectedAtTop", this::coralDetectedAtTopSensor, null); // s2
        builder.addBooleanProperty("coralDetectedAtBottom", this::coralDetectedAtBottomSensor, null); // s1
        builder.addBooleanProperty("rawTopSensor", () -> candi.getS1Closed().getValue(), null);
        builder.addBooleanProperty("rawBotSensor", () -> candi.getS2Closed().getValue(), null);
    }

    void intakeFast() {
        topMotor.setControl(dutyCycleOutTop.withOutput(intakeHighSpeedLeft));
        bottomMotor.setControl(dutyCycleOutBottom.withOutput(intakeHighSpeedRight));
    }

    void intakeSlow() {
        topMotor.setControl(dutyCycleOutTop.withOutput(intakeLowSpeed));
        bottomMotor.setControl(dutyCycleOutBottom.withOutput(-intakeLowSpeed));
    }

    private void eject() {
        topMotor.setControl(dutyCycleOutTop.withOutput(ejectSpeed));
        bottomMotor.setControl(dutyCycleOutBottom.withOutput(-ejectSpeed));
    }

    void stop() {
        topMotor.setControl(stop);
        bottomMotor.setControl(stop);
    }

    public boolean coralDetectedAtBottomSensor() {
        return bottomSensorDetected;
    }

    public boolean coralDetectedAtTopSensor() {
        return topSensorDetected;
    }


    public Command ejectCommand() {
        return this.run(this::eject);
    }

    public Command stopCommand() {
        return this.runOnce(this::stop).withName("Coral Stop");
    }

    public Command intakeCoralCommand() {
        return new IntakeCoralCommand();
    }

    public Command shootCoralCommand() {
        System.out.println("Intake Shoot Command Active");
        return new ShootCoralCommand().withTimeout(COMMAND_TIMEOUT_SECONDS);
    }

    public Command shootCoralInTroughCommand() {
        return this.run(() -> {
            topMotor.setControl(dutyCycleOutTop.withOutput(0.15));
            bottomMotor.setControl(dutyCycleOutBottom.withOutput(-0.25));
        });
    }

    private class ShootCoralCommand extends Command {
        public ShootCoralCommand() {
            addRequirements(CoralIntake.this);
        }

        @Override
        public void initialize() {
            CoralIntake.this.intakeFast();
        }

        @Override
        public void end(boolean interrupted) {
            CoralIntake.this.stop();
        }
    }

}