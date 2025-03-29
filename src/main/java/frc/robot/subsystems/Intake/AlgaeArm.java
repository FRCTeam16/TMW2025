package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.subsystems.Lifecycle;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

public class AlgaeArm extends SubsystemBase implements Lifecycle {

    public static final double ALLOWED_POSITION_ERROR = 0.1;

    private final TalonFX algaeArmMotor = new TalonFX(Robot.robotConfig.getCanID("algaeArmMotor"));
   private final CANcoder algaeArmEncoder = new CANcoder(Robot.robotConfig.getCanID("algaeArmEncoder"));

    private static final double GRAVITY_COMPENSATION = 0; //-0.49;

    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(0);
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(1);

    private double targetPosition = 0;
    private double openLoopMax = 0.3;


    public AlgaeArm() {
        CANcoderConfiguration encoderConfiguration = new CANcoderConfiguration()
            .withMagnetSensor(
                    new MagnetSensorConfigs()
                        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                        .withMagnetOffset(0.007812));
        algaeArmEncoder.getConfigurator().apply(encoderConfiguration);

        Slot0Configs slot0 = new Slot0Configs()
                .withKP(45)
                .withKI(0)
                .withKD(2)
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKG(0.5);

        Slot1Configs slot1 = new Slot1Configs()
                .withKP(60)
                .withKI(0)
                .withKD(3)
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKG(0.5);

        SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(0.240
        )
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(0.012);

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(0.6)
                .withMotionMagicAcceleration(1.8);

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
               .withFeedbackRemoteSensorID(Robot.robotConfig.getCanID("algaeArmEncoder"))
               .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder);

        TalonFXConfiguration armConfiguration = new TalonFXConfiguration()
                .withSlot0(slot0)
                .withSlot1(slot1)
                .withMotionMagic(motionMagicConfigs)
                .withMotorOutput(motorOutputConfigs)
                .withSoftwareLimitSwitch(softwareLimitSwitchConfigs)
                .withFeedback(feedbackConfigs);
        algaeArmMotor.getConfigurator().apply(armConfiguration);

        // this.algaeArmMotor.setPosition(0);
        this.setDefaultCommand(new DefaultHoldAlgaeArmCommand(this));
    }

    @Override
    public void teleopInit() {
        targetPosition = getMotorPosition();
    }

    @Override
    public void autoInit() {
        targetPosition = getMotorPosition();
    }

    public void setArmPosition(AlgaeArmPosition armPosition) {
        this.setArmPosition(armPosition.position);
    }

    private void setArmPosition(double position) {
        this.targetPosition = position;
        algaeArmMotor.setControl(
                motionMagic.withPosition(position)
                        .withFeedForward(calculateGravityCompensation(getEstimatedAngle())));
    }

    /**
     * Estimates the angle of the arm based on the current encoder position
     */
    public Angle getEstimatedAngle() {
        // Convert encoder ticks to rotations
        double rotations = getMotorPosition();

        // Convert rotations to degrees
        double degrees = 90.0 - (rotations / 0.23) * 90.0;

        return Degrees.of(degrees);
    }

    private double calculateGravityCompensation(Angle angle) {
        return Math.cos(angle.in(Radians)) * GRAVITY_COMPENSATION;
    }

    private void runOpenLoop(double speed) {
        double clampedSpeed = MathUtil.clamp(speed, -openLoopMax, openLoopMax);
        algaeArmMotor.setControl(dutyCycleOut.withOutput(clampedSpeed));
        this.targetPosition = algaeArmMotor.getPosition().getValueAsDouble();
    }

    public void holdPosition() {
        // double currentPosition = getMotorPosition();
        setArmPosition(targetPosition);
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
        builder.addBooleanProperty("isInPosition", this::isInPosition, null);
        builder.addDoubleProperty("estimatedAngle", () -> this.getEstimatedAngle().in(Degrees), null);

        if (Constants.DebugSendables.AlgaeArm) {
            builder.addDoubleProperty("openLoopMax", () -> openLoopMax, (v) -> openLoopMax = v);
        }
    }

    public Command openLoopCommand(Supplier<Double> speed) {
        return this.run(() -> runOpenLoop(speed.get())).withName("AlgaeArm Manual Control");
    }

//    public Command holdPositionCommand() {
//        return this.run(this::holdPosition).withName("Hold AlgaeArm Position");
//    }

    public Command setArmPositionCommand(AlgaeArmPosition position) {
        return new SetArmPositionCommand(position);
    }

    public static class DefaultHoldAlgaeArmCommand extends Command {
        public DefaultHoldAlgaeArmCommand(AlgaeArm algaeArm) {
            addRequirements(algaeArm);
        }

        @Override
        public void initialize() {
            Subsystems.algaeArm.holdPosition();
        }
    }


    public enum AlgaeArmPosition {
        Up(0.235),
        Ground(0.014),
        Processor(0.34),
        Shooting(0.220),
        PickFromReef(0.11);

        private final double position;

        AlgaeArmPosition(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public static class SetArmPositionCommand extends Command {
        private final AlgaeArm.AlgaeArmPosition position;


        public SetArmPositionCommand(AlgaeArm.AlgaeArmPosition position) {
            this.position = position;
            addRequirements(Subsystems.algaeArm);
        }

        @Override
        public void initialize() {
            Subsystems.algaeArm.setArmPosition(position.position);
        }

        @Override
        public boolean isFinished() {
            return Subsystems.algaeArm.isInPosition();
        }
    }

}

