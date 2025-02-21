package frc.robot.subsystems.Intake;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.Lifecycle;

public class CoralIntake extends SubsystemBase implements Lifecycle {
    public static final double COMMAND_TIMEOUT_SECONDS = 5.0;
    public static final int INVALID_LASER_MEASUREMENT = 9999;
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
    double intakeLowSpeed = 0.05;
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

    private int getLaser1Measurement() {
        Measurement measurement = laser1.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return (int) laser1Filter.calculate(measurement.distance_mm);
        } else {
            return INVALID_LASER_MEASUREMENT;
        }
    }

    private int getLaser2Measurement() {
        Measurement measurement = laser2.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return (int) laser2Filter.calculate(measurement.distance_mm);
        } else {
            return INVALID_LASER_MEASUREMENT;
        }
    }

    boolean coralDetectedAtFirstLaser() {
        int measurement = getLaser1Measurement();
        if (measurement < INVALID_LASER_MEASUREMENT) {
            return measurement < laser1SenseDistance;
        }
        return false;
    }

    boolean coralDetectedAtSecondLaser() {
        int measurement = getLaser2Measurement();
        if (measurement < INVALID_LASER_MEASUREMENT) {
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