package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radian;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PIDHelper;

    public class Elevator extends SubsystemBase implements Lifecycle {
    // looks like elevator is in fixed position so we dont have to deal with pivot pose
    // We may end up having a cancoder for the elevator, I'll assume we have them for now then make adjestments later if need be

    //  left.getPosition().getValue().in(Radian); is the equivalent to the CANcoder elevatorPose.getPosition().getValue().in(Radian);|

    public enum elevatatorSetpoint{ //TODO: these numbers will probably break things if ran on the bot but I need a robot built before we're able to fix them
        zero(0),
        TROUGH(0.0),    // Lowest position
        P1(0.0),        // POLE 1
        P2(0.0),        // POLE 2
        P3(0.0),        // POLE 3
        ALGAE(0.0),   // ALGAE BED
        twoPi(2*Math.PI);

        public final double val;
        private elevatatorSetpoint(double val){
            this.val = val;
        }
    }

    private TalonFX left;
    private TalonFX right;
    private CANcoder elevatorPose = new CANcoder(61);
    private double currentPoseAsRad; // refers to the number rotations of the cancoder in radians
    private double currentSetpoint;

    private NeutralOut brake = new NeutralOut();
    private DutyCycleOut runOpenLoop = new DutyCycleOut(0);
    private DutyCycleOut runClosedLoop = new DutyCycleOut(0);

    private PIDController elevatorPID = new PIDController(0, 0, 0);
    private PIDHelper elevatorHelper = new PIDHelper("Elevator/pid");

    private boolean isOpenloop = true; // DON'T TURN TO FALSE
    private double openLoopMotorOutput;
    private double closedLoopMotorOutput;

    public Elevator(){
        SmartDashboard.setDefaultNumber("Elevator/openLoopMotorOutput", 1);
        SmartDashboard.setDefaultNumber("Elevator/closedLoopMotorOutput", 0);
        SmartDashboard.setDefaultNumber("Elevator/leftMotorID", 160);
        SmartDashboard.setDefaultNumber("Elevator/rightMotorID", 161);

        updateMotorIds();

        elevatorHelper.initialize(0.01, 0, 0, 0, 0, 0); // TODO: real numbers
        elevatorPID.setTolerance(0.5); // TODO: real numbers
    }

    @Override
    public void periodic(){
        currentPoseAsRad = elevatorPose.getPosition().getValue().in(Radian);
        closedLoopMotorOutput = elevatorPID.calculate(currentPoseAsRad, currentSetpoint);
        runClosedLoop = new DutyCycleOut(closedLoopMotorOutput);

        if(!isOpenloop){
            right.setControl(runClosedLoop);
            left.setControl(runClosedLoop);
        }

        openLoopMotorOutput = SmartDashboard.getNumber("Elevator/openLoopMotorOutput", 0);
        SmartDashboard.setDefaultNumber("Elvator/closedLoopMotorOutput", closedLoopMotorOutput);
        SmartDashboard.updateValues();
       
    }

    public Command updateMotorIds(){
        return this.runOnce(() -> {
        left = new TalonFX((int)SmartDashboard.getNumber("Elevator/leftMotorID", 160));
        right = new TalonFX((int)SmartDashboard.getNumber("Elevator/rightMotorID", 161));
        });
    }

    public Command openLoopUp(){
        return this.runOnce(()->{
        runOpenLoop = new DutyCycleOut(openLoopMotorOutput);
        left.setControl(runOpenLoop);
        right.setControl(runOpenLoop);
        });
    }

    public Command openLoopDown(){
        return this.runOnce(()->{
        runOpenLoop = new DutyCycleOut(-openLoopMotorOutput);
        left.setControl(runOpenLoop);
        right.setControl(runOpenLoop);
        });
    }

    public Command openLoopStop(){
        return this.runOnce(() -> {
            left.setControl(brake);
            right.setControl(brake);
        });
    }
    public Command moveToPosition(elevatatorSetpoint e) {
                return this.runOnce(() -> {
                    currentSetpoint = e.val;
                    //    moveToPosition(val);
                    });
            }
        }