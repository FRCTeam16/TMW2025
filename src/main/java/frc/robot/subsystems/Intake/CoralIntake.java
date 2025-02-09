package frc.robot.subsystems.Intake;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.Lifecycle;
import frc.robot.util.BSLogger;

public class CoralIntake extends SubsystemBase implements Lifecycle {
    public static final double COMMAND_TIMEOUT_SECONDS = 5.0;
    private final TalonFX topMotor = new TalonFX(Robot.robotConfig.getCanID("coralIntakeTopMotor"));
    private final TalonFX bottomMotor  = new TalonFX(Robot.robotConfig.getCanID("coralIntakeBottomMotor"));;
    private final DutyCycleOut dutyCycleOutTop = new DutyCycleOut(1);
    private final DutyCycleOut dutyCycleOutBottom = new DutyCycleOut(1);
    private final LaserCan laser1 = new LaserCan(1);
    private final LaserCan laser2 = new LaserCan(2);
    private final NeutralOut stop = new NeutralOut();
    //TODO: GET REAL NUMS
    int laser1SenseDistance = 3;
    int laser2SenseDistance = 3;
    double intakeHighSpeed = 0.7;
    double intakeLowSpeed = 0.2;
    double ejectSpeed = -0.3;


    public CoralIntake() {
        this.setDefaultCommand(this.stopCommand());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("CoralIntake");
        builder.addDoubleProperty("intakeHighSpeed", () -> intakeHighSpeed, (v) -> intakeHighSpeed = v);
        builder.addDoubleProperty("intakeLowSpeed", () -> intakeLowSpeed, (v) -> intakeLowSpeed = v);
        builder.addDoubleProperty("ejectSpeed", () -> ejectSpeed, (v) -> ejectSpeed = v);
        builder.addBooleanProperty("coralDetectedAtFirstLaser", this::coralDetectedAtFirstLaser, null);
        builder.addBooleanProperty("coralDetectedAtSecondLaser", this::coralDetectedAtSecondLaser, null);
    }

    private void intakeFast() {
        topMotor.setControl(dutyCycleOutTop.withOutput(intakeHighSpeed));
        bottomMotor.setControl(dutyCycleOutBottom.withOutput(intakeHighSpeed));
    }

    private void intakeSlow() {
        topMotor.setControl(dutyCycleOutTop.withOutput(intakeLowSpeed));
        bottomMotor.setControl(dutyCycleOutBottom.withOutput(intakeLowSpeed));
    }

    private void eject() {
        topMotor.setControl(dutyCycleOutTop.withOutput(ejectSpeed));
        bottomMotor.setControl(dutyCycleOutBottom.withOutput(ejectSpeed));
    }

    private void stop() {
        topMotor.setControl(stop);
        bottomMotor.setControl(stop);
    }

    private boolean coralDetectedAtFirstLaser() {
        Measurement measurement = laser1.getMeasurement();
        if (measurement != null) {
            return measurement.distance_mm > laser1SenseDistance;
        }
        return false;
    }

    private boolean coralDetectedAtSecondLaser() {
        Measurement measurement = laser2.getMeasurement();
        if (measurement != null) {
            return measurement.distance_mm > laser2SenseDistance;
        }
        return false;
    }


    public Command ejectCommand() {
        return this.runOnce(this::eject);
    }

    public Command stopCommand() {
        return this.runOnce(this::stop);
    }

    public Command intakeCoralCommand() {
        return new IntakeCoralCommand().withTimeout(COMMAND_TIMEOUT_SECONDS);
    }

    public Command shootCoralCommand() {
        return new ShootCoralCommand().withTimeout(COMMAND_TIMEOUT_SECONDS);
    }

    public class IntakeCoralCommand extends Command {
        int step = 1;
        boolean shooting = false;

        public IntakeCoralCommand() {
            addRequirements(CoralIntake.this);
        }

        @Override
        public void initialize() {
            // Start with fast intake
            CoralIntake.this.intakeFast();
        }

        @Override
        public void execute() {
            //default action when intake: runForward(fast)
            if (step == 1) {
                //if first laser sees coral while default action: change action to action 2
                if (coralDetectedAtFirstLaser()) {
                    BSLogger.log("CoralIntakeCommand", "Coral detected at first laser");
                    CoralIntake.this.intakeSlow();
                    step = 2;
                }
            }

            //second action when intake: runForward(slow)
            if (step == 2) {
                //if second laser sees coral while second action: change action to action 3
                if (coralDetectedAtSecondLaser()) {
                    BSLogger.log("CoralIntakeCommand", "Coral detected at second laser");
                    CoralIntake.this.stop();
                    step = 3;
                }
            }
        }

        @Override
        public boolean isFinished() {
            return coralDetectedAtSecondLaser() || step == 3;
        }
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
        public boolean isFinished() {
            return !coralDetectedAtSecondLaser();
        }

        @Override
        public void end(boolean interrupted) {
            CoralIntake.this.stop();
        }
    }
}