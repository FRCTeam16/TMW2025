package frc.robot.subsystems.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Lifecycle;
import frc.robot.util.BSLogger;

public class FunnelSubsystem extends SubsystemBase implements Lifecycle {
    private final Latch latchImpl = new ActuatorLatch();
    private final PWM funnelLatch = latchImpl.getActuator();
    private double setpoint = 0.0;
    private boolean latchOpened = false; // assume we start true

    public FunnelSubsystem() {
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("FunnelSubsystem");
        builder.addBooleanProperty("Latch Opened", () -> latchOpened, null);

        if (Constants.DebugSendables.Funnel) {
            builder.addDoubleProperty("Position", funnelLatch::getPosition, null);
            builder.addBooleanProperty("In Position", this::inPosition, null);
        }
    }

    public boolean inPosition() {
        return MathUtil.isNear(setpoint, funnelLatch.getPosition(), 0.01);
    }

    private void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
        funnelLatch.setPosition(this.setpoint);
    }

    public void openLatch() {
        BSLogger.log("Funnel", "openLatch");
        latchOpened = true;
        this.setSetpoint(latchImpl.getOpenPosition());
    }

    public void closeLatch() {
        BSLogger.log("Funnel", "closeLatch");
        latchOpened = false;
        this.setSetpoint(latchImpl.getClosedPosition());
    }

    /**
     * Opens the latch causing the funnel to drop
     */
    public Command openLatchCommand() {
        return this.runOnce(this::openLatch);
    }

    public Command closeLatchCommand() {
        return this.runOnce(this::closeLatch);
    }

    public boolean isLatchOpen() {
        return latchOpened;
    }

    interface Latch {
        double getOpenPosition();
        double getClosedPosition();
        PWM getActuator();
    }

    /**
     * This is a Servo latch that is used to control the funnel servo.
     */
    static class ServoLatch implements Latch {
        final Servo actuator = new Servo(0);

        @Override
        public double getOpenPosition() {
            return 1.0;
        }

        @Override
        public double getClosedPosition() {
            return 0.1;
        }

        @Override
        public PWM getActuator() {
            return actuator;
        }

    }

    /**
     * This is a PWM latch that is used to control the funnel linear actuator.
     */
    static class ActuatorLatch implements Latch{
        final PWM actuator = new PWM(0);

        @Override
        public double getOpenPosition() {
            return 0.0;
        }

        @Override
        public double getClosedPosition() {
            return 1.0;
        }

        @Override
        public PWM getActuator() {
            return actuator;
        }
    }
}
