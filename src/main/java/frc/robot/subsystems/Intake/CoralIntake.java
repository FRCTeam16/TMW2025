package frc.robot.subsystems.Intake;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.Lifecycle;
import frc.robot.util.BSLogger;

import static edu.wpi.first.units.Units.Millimeters;

public class CoralIntake extends SubsystemBase implements Lifecycle {
    public static final double COMMAND_TIMEOUT_SECONDS = 5.0;
    private final TalonFX topMotor = new TalonFX(Robot.robotConfig.getCanID("coralIntakeLeftMotor"));
    private final TalonFX bottomMotor  = new TalonFX(Robot.robotConfig.getCanID("coralIntakeRightMotor"));;
    private final DutyCycleOut dutyCycleOutTop = new DutyCycleOut(1);
    private final DutyCycleOut dutyCycleOutBottom = new DutyCycleOut(1);
    private final LaserCan laser1 = new LaserCan(1);    // top
    private final LaserCan laser2 = new LaserCan(2);    // bottom

    private final MedianFilter laser1Filter = new MedianFilter(5);
    private final MedianFilter laser2Filter = new MedianFilter(5);

    private final StaticBrake stop = new StaticBrake();
    

    //TODO: GET REAL NUMS
    int laser1SenseDistance = 30;
    int laser2SenseDistance = 30;
    double intakeHighSpeedLeft = 0.25;
    double intakeHighSpeedRight = -0.25;
    double intakeLowSpeed = 0.2;
    double ejectSpeed = -0.3;


    public CoralIntake() {
        this.setDefaultCommand(this.stopCommand());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("CoralIntake");
        builder.addDoubleProperty("intakeHighSpeedLeft", () -> intakeHighSpeedLeft, (v) -> intakeHighSpeedLeft = v);
        builder.addDoubleProperty("intakeHighSpeedRight", () -> intakeHighSpeedRight, (v) -> intakeHighSpeedRight = v);

        builder.addDoubleProperty("intakeLowSpeed", () -> intakeLowSpeed, (v) -> intakeLowSpeed = v);
        builder.addDoubleProperty("ejectSpeed", () -> ejectSpeed, (v) -> ejectSpeed = v);
        builder.addBooleanProperty("coralDetectedAtFirstLaser", this::coralDetectedAtFirstLaser, null);
        builder.addIntegerProperty("laser1 Dist", this::getLaser1Measurement, null);
        builder.addBooleanProperty("coralDetectedAtSecondLaser", this::coralDetectedAtSecondLaser, null);
        builder.addIntegerProperty("laser2 Dist", this::getLaser2Measurement, null);
    }

    private void intakeFast() {
        topMotor.setControl(dutyCycleOutTop.withOutput(intakeHighSpeedLeft));
        bottomMotor.setControl(dutyCycleOutBottom.withOutput(intakeHighSpeedRight));
    }

    private void intakeSlow() {
        topMotor.setControl(dutyCycleOutTop.withOutput(intakeLowSpeed));
        bottomMotor.setControl(dutyCycleOutBottom.withOutput(-intakeLowSpeed));
    }

    private void eject() {
        topMotor.setControl(dutyCycleOutTop.withOutput(ejectSpeed));
        bottomMotor.setControl(dutyCycleOutBottom.withOutput(-ejectSpeed));
    }

    private void stop() {
        topMotor.setControl(stop);
        bottomMotor.setControl(stop);
    }

    private int getLaser1Measurement() {
        Measurement measurement = laser1.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return (int) laser1Filter.calculate(measurement.distance_mm);
        } else {
            return 9999;
        }
    }

    private int getLaser2Measurement() {
        Measurement measurement = laser2.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return (int) laser2Filter.calculate(measurement.distance_mm);
        } else {
            return 9999;
        }
    }

    private boolean coralDetectedAtFirstLaser() {
        int measurement = getLaser1Measurement();
        if (measurement < 9999) {
            return measurement < laser1SenseDistance;
        }
        return false;
    }

    private boolean coralDetectedAtSecondLaser() {
        int measurement = getLaser2Measurement();
        if (measurement < 9999) {
            return measurement < laser2SenseDistance;
        }
        return false;
    }


    public Command ejectCommand() {
        System.out.println("Intake Eject Command Active");
        return this.run(this::eject);
    }

    public Command stopCommand() {
        return this.runOnce(this::stop).withName("Coral Stop");
    }

    public Command intakeCoralCommand() {
        return new IntakeCoralCommand().withTimeout(10);
    }

    public Command shootCoralCommand() {
        System.out.println("Intake Shoot Command Active");
        return new ShootCoralCommand().withTimeout(COMMAND_TIMEOUT_SECONDS);
    }

    public Command shootCoralInTroughCommand() {
        return Commands.run(() -> {
            topMotor.setControl(dutyCycleOutTop.withOutput(0.15));
            bottomMotor.setControl(dutyCycleOutBottom.withOutput(-0.25));
        });
    }

    public class IntakeCoralCommand extends Command {
        int step = 1;
        boolean shooting = false;

        public IntakeCoralCommand() {
            addRequirements(CoralIntake.this);
        }

        @Override
        public void initialize() {
            BSLogger.log("CoralIntakeCommand", "**** STARTING ****");
            step = 1;   // FIXME: Investigate in debugger
            // Start with fast intake
            CoralIntake.this.intakeFast();
        }

        @Override
        public void execute() {
            BSLogger.log("CoralIntakeCommand", "**** EXECUTING CORAL INTAKE STEP: " + step);
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
//            return !coralDetectedAtSecondLaser();
            return false;
        }

        @Override
        public void end(boolean interrupted) {
            CoralIntake.this.stop();
        }
    }

}