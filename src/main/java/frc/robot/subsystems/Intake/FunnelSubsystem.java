package frc.robot.subsystems.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Lifecycle;
import frc.robot.util.BSLogger;

public class FunnelSubsystem extends SubsystemBase implements Lifecycle {
    private PWM funnelLatch = new PWM(0);
    private double setpoint = 0.0;

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.setSmartDashboardType("FunnelSubsystem");
        builder.addDoubleProperty("Position", funnelLatch::getPosition, null);;
        builder.addDoubleProperty("Setpoint", () -> setpoint, this::setSetpoint);
        builder.addBooleanProperty("In Position", this::inPosition, null);
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
        this.setSetpoint(1.0);
    }

    public void closeLatch() {
        BSLogger.log("Funnel", "closeLatch");
        this.setSetpoint(0.0);
    }

    public Command openLatchCommand() {
        return this.runOnce(this::openLatch);
    }


    public Command closeLatchCommand() {
        return this.runOnce(this::closeLatch);
    }

}
