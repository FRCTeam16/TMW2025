package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.commands.amd.AlgaeIntakeAMDCommand;
import frc.robot.subsystems.AMD;
import frc.robot.subsystems.Lifecycle;
import frc.robot.util.MotorStatorCurrentFilter;

import static edu.wpi.first.units.Units.Amps;

public class AlgaeIntake extends SubsystemBase implements Lifecycle, AMD<AlgaeIntakeAMDCommand.AlgaeIntakeDataCollector> {
    private final TalonFX algaeIntakeMotor = new TalonFX(Robot.robotConfig.getCanID("algaeIntakeMotor"));
    private final NeutralOut brake = new NeutralOut();
    private final DutyCycleOut intakeDutyCycleOut = new DutyCycleOut(1);
    private final MotorStatorCurrentFilter detector = new MotorStatorCurrentFilter(Amps.of(30),
            algaeIntakeMotor.getStatorCurrent().asSupplier());
    private final MotorStatorCurrentFilter dropDetector = new MotorStatorCurrentFilter(Amps.of(-20),
            algaeIntakeMotor.getStatorCurrent().asSupplier());


    private double forwardSpeed = 0.6;
    private double backwardSpeed = -0.3;
    private double holdSpeed = 0.04;
    private boolean algaeDetected = false;
    private String requestedState = "None";

    public AlgaeIntake() {
        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake);
        TalonFXConfiguration intakeConfiguration = new TalonFXConfiguration()
                .withMotorOutput(motorOutputConfigs);
        algaeIntakeMotor.getConfigurator().apply(intakeConfiguration);

        this.setDefaultCommand(new DefaultHoldCommand());
    }

    @Override
    public void periodic() {
        detector.update();
        if (detector.isOverThreshold()) {
            algaeDetected = true;
        }

        dropDetector.update();
        if (algaeDetected && dropDetector.isUnderThreshold()) {
            algaeDetected = false;
        }
    }

    public boolean isAlgaeDetected() {
        return this.algaeDetected;
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
        builder.addStringProperty("requestedState", () -> requestedState, null);
        builder.addBooleanProperty("algaeDetected", () -> algaeDetected, null);

        if (Constants.DebugSendables.AlgaeIntake) {
            builder.addDoubleProperty("filteredCurrent", () -> detector.getCurrent().in(Amps), null);
            builder.addBooleanProperty("filteredThreshold", detector::isOverThreshold, null);

            builder.addDoubleProperty("forwardSpeed", () -> forwardSpeed, this::setForwardSpeed);
            builder.addDoubleProperty("backwardSpeed", () -> backwardSpeed, this::setBackwardSpeed);
            builder.addDoubleProperty("holdSpeed", () -> holdSpeed, this::setHoldSpeed);
        }
    }

    public void intakeAlgae() {
        algaeIntakeMotor.setControl(intakeDutyCycleOut.withOutput(forwardSpeed));
    }

    public void holdAlgae() {
        algaeIntakeMotor.setControl(intakeDutyCycleOut.withOutput(holdSpeed));
    }

    public void runAMD() {
        algaeIntakeMotor.setControl(intakeDutyCycleOut.withOutput(forwardSpeed));
    }

    public void stopAlgae() {
        algaeIntakeMotor.setControl(brake);
    }

    public Command intakeCommand() {
        return this.run(this::intakeAlgae)
                .alongWith(Commands.runOnce(() -> requestedState = "Intake"))
                .withName("Algae Intake");
    }

    public Command ejectCommand() {
        return this.startRun(
            () -> this.algaeDetected = false,
            () -> algaeIntakeMotor.setControl(intakeDutyCycleOut.withOutput(backwardSpeed))
            ).alongWith(Commands.runOnce(() -> requestedState = "Eject"))
            .withName("Algae Eject");

    }

    public Command holdAlgaeCommand() {
        return this.run(this::holdAlgae)
                .alongWith(Commands.runOnce(() -> requestedState = "Hold"))
                .withName("Algae Hold");

    }

    public Command stopCommand() {
        return this.run(this::stopAlgae)
                .alongWith(Commands.runOnce(() -> requestedState = "Stop"))
                .withName("Algae Stop");
    }

    @Override
    public void collectAMDData(AlgaeIntakeAMDCommand.AlgaeIntakeDataCollector dataCollector) {
        dataCollector.addCurrents(algaeIntakeMotor.getStatorCurrent().getValueAsDouble());
    }

    class DefaultHoldCommand extends Command {
        DefaultHoldCommand() {
            addRequirements(AlgaeIntake.this);
            setName("Algae Hold");
        }

        @Override
        public void initialize() {
            requestedState = "Hold";
            algaeIntakeMotor.setControl(brake);
        }
    }

    class PulseHoldAlgaeCommand extends Command {
        private final Timer timer = new Timer();
        private boolean pulseState = false;

        private final DutyCycleOut pulse = new DutyCycleOut(holdSpeed);
        private final StaticBrake brake = new StaticBrake();

        public PulseHoldAlgaeCommand() {
            addRequirements(Subsystems.algaeIntake);
            setName("Pulse Hold");
        }

        @Override
        public void initialize() {
            timer.restart();
            pulseState = true;
        }

        @Override
        public void execute() {
            if (pulseState) {
                algaeIntakeMotor.setControl(pulse);
                if (timer.hasElapsed(1.0)) {
                    pulseState = false;
                }
            } else {
                algaeIntakeMotor.setControl(brake);
                if (timer.hasElapsed(0.1)) {
                    pulseState = true;
                }
            }
//            if (timer.hasElapsed(0.5)) {
//                if (pulseState) {
//                    algaeIntakeMotor.setControl(pulse);
//                } else {
//                    algaeIntakeMotor.setControl(brake);
//                }
//                pulseState = !pulseState;
//            }
        }

        @Override
        public void end(boolean interrupted) {
            algaeIntakeMotor.setControl(brake);
        }
    }
}
