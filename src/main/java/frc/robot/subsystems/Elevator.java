package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Elevator extends SubsystemBase implements Lifecycle {
    public static final double GRAVITY_VOLTS = -0.5;
    private final TalonFX left = new TalonFX(Robot.robotConfig.getCanID("elevatorLeftMotor"));
    private final TalonFX right = new TalonFX(Robot.robotConfig.getCanID("elevatorRightMotor"));
    private final NeutralOut neutralOut = new NeutralOut();
    private final PositionVoltage positionVoltage = new PositionVoltage(0);
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final MotionMagicVoltage motionMagicV = new MotionMagicVoltage(0).withFeedForward(Units.Volts.of(GRAVITY_VOLTS));
    private final StaticBrake staticBrake = new StaticBrake();
    private double openLoopMotorSpeed = -0.2;
    private double currentSetpoint = 0;


    public Elevator() {
        right.setControl(new Follower(left.getDeviceID(), true));

        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = 2.5; //2.5
        slot0.kI = 0.0;
        slot0.kD = 0.0;
        slot0.kG = GRAVITY_VOLTS; // -0.75

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = 80;
        motionMagicConfigs.MotionMagicAcceleration = 140;
        motionMagicConfigs.MotionMagicJerk = 0;
        motionMagicConfigs.MotionMagicExpo_kA = 0.0;
        motionMagicConfigs.MotionMagicExpo_kV = 0.01;

        TalonFXConfiguration configuration = new TalonFXConfiguration()
                .withSlot0(slot0)
                .withMotionMagic(motionMagicConfigs)
                .withSoftwareLimitSwitch(
                        new SoftwareLimitSwitchConfigs()
                                .withForwardSoftLimitEnable(true)
                                .withForwardSoftLimitThreshold(0.0)
                                .withReverseSoftLimitEnable(true)
                                .withReverseSoftLimitThreshold(-150.0)
                );

        left.getConfigurator().apply(configuration);

        // Zero motor position at startup
        left.setPosition(0);
    }

    private void moveToPosition(ElevatorSetpoint setpoint) {
        this.currentSetpoint = setpoint.val;
        left.setControl(motionMagicV.withPosition(setpoint.val));
    }

    /**
     * Returns the position of the elevator in motor rotations
     *
     * @return the position of the elevator in motor rotations
     */
    private double getCurrentPosition() {
        return left.getPosition().getValueAsDouble();
    }

    public boolean isInPosition() {
        return Math.abs(getCurrentPosition() - currentSetpoint) < 0.1;
    }

    public Command openLoopUpCommand() {
        return this.runOnce(() -> left.setControl(dutyCycleOut.withOutput(openLoopMotorSpeed))).withName("Open Loop Up");
    }

    public Command openLoopDownCommand() {
        return this.runOnce(() -> left.setControl(dutyCycleOut.withOutput(-openLoopMotorSpeed))).withName("Open Loop Down");
    }

    /**
     * Moves the elevator to the current setpoint value, primarily used for testing
     *
     * @return a command that moves the elevator to the current setpoint value
     */
    public Command moveToCurrentSetpoint() {
        return this.runOnce(() -> left.setControl(motionMagicV.withPosition(currentSetpoint))).withName("Move to Current Setpoint");
    }

    /**pilibj.RobotBase.runRobot(RobotBase.
     * Applies a neutral output to the elevator motors
     *
     * @return a command that applies a neutral output to the elevator motors
     */
    public Command openLoopStopCommand() {
        return this.runOnce(() -> left.setControl(neutralOut)).withName("Open Loop Stop");
    }

    public Command holdPositionCommand() {
        // TODO: Need to test just using static brake
        return this.runOnce(() -> left.setControl(motionMagicV.withPosition(getCurrentPosition()))).withName("Hold Position");
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("Elevator");
        builder.addDoubleProperty("Current Position", this::getCurrentPosition, null);
        builder.addBooleanProperty("Is In Position", this::isInPosition, null);
        builder.addDoubleProperty("Current Setpoint", () -> currentSetpoint, (sp) -> currentSetpoint = sp);

        builder.addDoubleProperty("Open Loop Motor Speed", () -> openLoopMotorSpeed, (speed) -> openLoopMotorSpeed = speed);
        builder.addDoubleProperty("Left Motor Volts", () -> left.getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("Left Motor DutyCycle", () -> left.getDutyCycle().getValueAsDouble(), null);
        builder.addDoubleProperty("Left Motor Current", () -> left.getStatorCurrent().getValueAsDouble(), null);
        builder.addDoubleProperty("Right Motor Current", () -> right.getStatorCurrent().getValueAsDouble(), null);
    }

    public enum ElevatorSetpoint { //TODO: these numbers will probably break things if ran on the bot but I need a robot built before we're able to fix them
        Zero(0), TROUGH(0.0),    // Lowest position
        P1(0.0),        // POLE 1
        P2(0.0),        // POLE 2
        P3(0.0),        // POLE 3
        AlgaeBarge(0.0),        // ALGAE BED
        AlgaeProcessor(0.0),    // ALGAE Processor
        AlgaeReefHigh(0.0),     // ALGAE Reef High
        AlgaeReefLow(0.0),      // ALGAE Reef Low
        twoPi(2 * Math.PI);

        // -150 soft lim during testing

        public final double val;

        ElevatorSetpoint(double val) {
            this.val = val;
        }
    }


    public class ElevatorMoveToPositionCommand extends Command {
        private final ElevatorSetpoint setpoint;

        public ElevatorMoveToPositionCommand(ElevatorSetpoint setpoint) {
            this.setpoint = setpoint;
            addRequirements(Elevator.this);
        }

        @Override
        public void initialize() {
            Elevator.this.moveToPosition(this.setpoint);
        }

        @Override
        public boolean isFinished() {
            return Elevator.this.isInPosition();
        }
    }

}