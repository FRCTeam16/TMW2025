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
import frc.robot.util.BSLogger;

import static frc.robot.subsystems.Intake.IntakeCoralCommand.STOP_CORAL_INTAKE_TASK;

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


    double intakeHighSpeedLeft = 0.25;
    double intakeHighSpeedRight = -0.25;
    double intakeLowSpeed = 0.2;
    double ejectSpeed = -0.3;
    private boolean topSensorDetected;
    private boolean bottomSensorDetected;


    private boolean coralIntakeMode = false;
    private int coralIntakeStep = 1;


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

    @Override
    public void teleopInit() {
        coralIntakeMode = false;
        this.setDefaultCommand(this.stopCommand());
    }

    @Override
    public void autoInit() {
        this.removeDefaultCommand();
    }

    private void updateDetectorState() {
        topSensorDetected = topSensorFilter.calculate(candi.getS1Closed().getValue());
        bottomSensorDetected = bottomSensorFilter.calculate(candi.getS2Closed().getValue());
    }

    public void startIntakeAuto() {
        if (!coralIntakeMode) {
            this.intakeFast();
            coralIntakeMode = true;
            coralIntakeStep = 1;
            Subsystems.asyncManager.register(STOP_CORAL_INTAKE_TASK, () -> {
                if (coralIntakeStep < 3 && Subsystems.coralIntake.coralDetectedAtBottomSensor()) {
                    BSLogger.log("IntakeCoralCommandAsync", "Async STOPPING INTAKE CMD");
                    coralIntakeStep = 3;
                    Subsystems.coralIntake.stop();
                }
            });
        }
        coralIntakeMode = true;
    }

    public void stopIntakeAuto() {
        Subsystems.asyncManager.unregister(STOP_CORAL_INTAKE_TASK);
        coralIntakeMode = false;
        this.stop();
    }

    @Override
    public void periodic() {
        if (coralIntakeMode) {
            if (coralIntakeStep == 1) {
                //if first laser sees coral while default action: change action to action 2
                if (this.coralDetectedAtTopSensor()) {
                    BSLogger.log("CoralIntakeCommand", "Coral detected at first sensor");
                    this.intakeSlow();
                    coralIntakeStep = 2;
                }
            }

            //second action when intake: runForward(slow)
            if (coralIntakeStep == 2) {
                //if second laser sees coral while second action: change action to action 3
                if (this.coralDetectedAtBottomSensor()) {
                    BSLogger.log("CoralIntakeCommand", "Coral detected at second sensor");
                    this.stop();
                    coralIntakeStep = 3;
                }
            }
        }
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
        boolean startedWithSensor = false;

        public ShootCoralCommand() {
            addRequirements(CoralIntake.this);
        }

        @Override
        public void initialize() {
            startedWithSensor = coralDetectedAtBottomSensor();
            CoralIntake.this.intakeFast();
        }

        @Override
        public boolean isFinished() {
            if (startedWithSensor) {
                return !coralDetectedAtBottomSensor();
            }
            return false;
        }

        @Override
        public void end(boolean interrupted) {
            CoralIntake.this.stop();
        }
    }

}