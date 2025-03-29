package frc.robot.subsystems.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Lifecycle;
import frc.robot.util.BSLogger;

public class FunnelSubsystem extends SubsystemBase implements Lifecycle {
    private final Servo funnelLatch = new Servo(0);
    private double setpoint = 0.0;
    private boolean latchOpened = true; // assume we start true

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
        this.setSetpoint(1.0);
    }

    public void closeLatch() {
        BSLogger.log("Funnel", "closeLatch");
        latchOpened = false;
        this.setSetpoint(0.1);
    }

    public Command openLatchCommand() {
        return this.runOnce(this::openLatch);
    }

    public Command closeLatchCommand() {
        return this.runOnce(this::closeLatch);
    }

    public boolean isLatchOpen() {
        return latchOpened;
    }
}
