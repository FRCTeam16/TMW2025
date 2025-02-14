package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.BSLogger;

import static edu.wpi.first.units.Units.Radian;

public class Elevator extends SubsystemBase implements Lifecycle {
    // We may end up having a cancoder for the elevator, I'll assume we have them for now then make adjestments later if need be

    //  left.getPosition().getValue().in(Radian); is the equivalent to the CANcoder elevatorPose.getPosition().getValue().in(Radian);|

    private final CANcoder elevatorPose = new CANcoder(61);
    private final NeutralOut brake = new NeutralOut();
    private final PositionVoltage positionVoltage = new PositionVoltage(0);
    private TalonFX left;
    private TalonFX right;
    private double currentPoseAsRad;
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final MotionMagicVoltage motionMagicV = new MotionMagicVoltage(0).withFeedForward(Units.Volts.of(-0.5));
    private double openLoopMotorSpeed = 0.2;
    private double currentSetpoint = 0;

    public Elevator() {
        if (true) {
            left = new TalonFX(Robot.robotConfig.getCanID("elevatorLeftMotor"));
            right = new TalonFX(Robot.robotConfig.getCanID("elevatorRightMotor"));
            right.setControl(new Follower(left.getDeviceID(), true));

            Slot0Configs slot0 = new Slot0Configs();
            slot0.kP = 2.5; //2.5
            slot0.kI = 0.0;
            slot0.kD = 0.0;
            slot0.kG = -0.75; // -0.75

            MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
            motionMagicConfigs.MotionMagicCruiseVelocity = 80;
            motionMagicConfigs.MotionMagicAcceleration = 140;
            motionMagicConfigs.MotionMagicJerk = 0;
            motionMagicConfigs.MotionMagicExpo_kA = 0.0;
            motionMagicConfigs.MotionMagicExpo_kV = 0.01;

            TalonFXConfiguration configuration = new TalonFXConfiguration().withSlot0(slot0).withMotionMagic(motionMagicConfigs);

            left.getConfigurator().apply(configuration);
            
            // Zero motor position at startup
            left.setPosition(0);
        }
    }

    private void moveToPosition(ElevatorSetpoint setpoint) {
        this.currentSetpoint = setpoint.val;
        left.setControl(positionVoltage.withPosition(setpoint.val));
    }

    private double getCurrentPosition() {
        return left.getPosition().getValueAsDouble();
        //return 0;
    }

    public boolean isInPosition() {
        return Math.abs(getCurrentPosition() - currentSetpoint) < 0.1;
    }


    public Command openLoopUpCommand() {
        return this.runOnce(() -> {
            left.setControl(dutyCycleOut.withOutput(openLoopMotorSpeed));
        });
    }
    
    // left.getPosition().getValue().in(Radian);

    public Command openLoopDownCommand() {
        return this.runOnce(() -> {
            left.setControl(dutyCycleOut.withOutput(-openLoopMotorSpeed));
        });
    }

    public Command openLoopStopCommand() {
        return this.runOnce(() -> {
            left.setControl(brake);
        });
    }

    public Command holdPositionCommand() {
        return this.runOnce(() -> {
            left.setControl(positionVoltage.withPosition(getCurrentPosition()));
        });
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("Elevator");
        builder.addDoubleProperty("Current Position", this::getCurrentPosition, null);
        builder.addBooleanProperty("Is In Position", this::isInPosition, null);
        builder.addDoubleProperty("Current Setpoint", () -> currentSetpoint, (sp) -> currentSetpoint = sp);
        builder.addDoubleProperty("Open Loop Motor Speed", () -> openLoopMotorSpeed, (speed) -> openLoopMotorSpeed = speed);
        builder.addDoubleProperty("Motor Volts", () -> left.getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("Motor DutyCycle", () -> left.getDutyCycle().getValueAsDouble(), null);
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