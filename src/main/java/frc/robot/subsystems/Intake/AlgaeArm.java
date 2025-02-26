package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.subsystems.Lifecycle;
import frc.robot.util.BSLogger;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

public class AlgaeArm extends SubsystemBase implements Lifecycle {

    public static final double ALLOWED_POSITION_ERROR = 0.1;

    private final TalonFX algaeArmMotor = new TalonFX(Robot.robotConfig.getCanID("algaeArmMotor"));

    private final TalonFXConfiguration armConfiguration;

    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);

    private double targetPosition = 0;
    private double openLoopMax = 0.3;


    public AlgaeArm() {
        armConfiguration = new TalonFXConfiguration();
        armConfiguration.Slot0.kP = 0.6;
        armConfiguration.Slot0.kI = 0;
        armConfiguration.Slot0.kD = 0;
        armConfiguration.Slot0.kG = -0.49; // Gravity compensation
        armConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0;
        armConfiguration.MotionMagic.MotionMagicAcceleration = 0;
        algaeArmMotor.getConfigurator().apply(armConfiguration);
        algaeArmMotor.setNeutralMode(NeutralModeValue.Brake);

        this.setDefaultCommand(this.holdPositionCommand());
    }

    @Override
    public void teleopInit() {
        this.setDefaultCommand(this.holdPositionCommand());
    }

    private void setArmPosition(double position) {
        // TODO: Consider a method to allow pass in of translated angles?
//        BSLogger.log("AlgaeArm", "Setting position to: " + position + " | Estimated angle: " + getEstimatedAngle());
        this.targetPosition = position;
        // TODO: Check for near zero here and don't do anything, assume we are held
//        algaeArmMotor.setControl(
//                positionVoltage.withPosition(position)
//                        .withFeedForward(calculateGravityCompensation(getEstimatedAngle())));
        algaeArmMotor.setControl(new NeutralOut());
    }

    /**
     * Estimates the angle of the arm based on the current encoder position
     */
    public Angle getEstimatedAngle() {
        // Convert encoder ticks to rotations
        double rotations = getMotorPosition();

        // Convert rotations to degrees
        double degrees = 90.0 - (rotations / 8.6) * 90.0;

        return Degrees.of(degrees);
    }

    private double calculateGravityCompensation(Angle angle) {
        return Math.cos(angle.in(Radians)) * armConfiguration.Slot0.kG;
    }

    private void runOpenLoop(double speed) {
        BSLogger.log("AlgaeArm", "Running open loop with speed: " + speed);
        double clampedSpeed = MathUtil.clamp(speed, -openLoopMax, openLoopMax);
        algaeArmMotor.setControl(dutyCycleOut.withOutput(clampedSpeed));
    }

    public void holdPosition() {
        double currentPosition = getMotorPosition();
        setArmPosition(currentPosition);
    }

    private void setPosition(AlgaeArmPosition position) {
        setArmPosition(position.getPosition());
    }

    public boolean isInPosition() {
        // TODO: Test using algaeArmMotor.getClosedLoopError() if using magic motion
//        if (ControlModeValue.DutyCycleOut == algaeArmMotor.getControlMode().getValue()) {
//            return algaeArmMotor.getClosedLoopError().getValueAsDouble() < ALLOWED_POSITION_ERROR;
//        }
        return Math.abs(getMotorPosition() - targetPosition) < ALLOWED_POSITION_ERROR;
    }

    public double getMotorPosition() {
        return algaeArmMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("AlgaeArm");
        builder.addDoubleProperty("motorPosition", this::getMotorPosition, this::setArmPosition);
        builder.addDoubleProperty("targetPosition", () -> this.targetPosition, this::setArmPosition);
        builder.addDoubleProperty("estimatedAngle", () -> this.getEstimatedAngle().in(Degrees), null);
        builder.addBooleanProperty("isInPosition", this::isInPosition, null);
        builder.addDoubleProperty("openLoopMax", () -> openLoopMax, (v) -> openLoopMax = v);
    }

    public Command openLoopCommand(Supplier<Double> speed) {
        return this.run(() -> runOpenLoop(speed.get())).withName("AlgaeArm Manual Control");
    }

    public Command holdPositionCommand() {
        return this.run(this::holdPosition).withName("Hold AlgaeArm Position");
    }

    public Command setArmPositionCommand(AlgaeArmPosition position) {
        return new SetArmPositionCommand(position);
    }

    public class DefaultHoldAlgaeArmCommand extends Command {
        public DefaultHoldAlgaeArmCommand() {
            addRequirements(Subsystems.algaeArm);
        }
    }


    public enum AlgaeArmPosition {
        Start(0.0),
        ReefLow(5.0),
        ReefHigh(6.0),
        Processor(0.0),
        Shooting(3.0);

        private final double position;

        AlgaeArmPosition(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public class SetArmPositionCommand extends Command {
        private final AlgaeArm.AlgaeArmPosition position;

        public SetArmPositionCommand(AlgaeArm.AlgaeArmPosition position) {
            this.position = position;
            addRequirements(AlgaeArm.this);
        }

        @Override
        public void initialize() {
            AlgaeArm.this.setArmPosition(position.position);
        }

        @Override
        public boolean isFinished() {
            return AlgaeArm.this.isInPosition();
        }
    }

}

