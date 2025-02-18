package frc.robot.hci.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ZeroYawCommand;
import frc.robot.commands.vision.PipelineSwitcher;
import frc.robot.subsystems.vision.Pipeline;

public class CompBotControls extends ControlBinding {

    private final Trigger requestL1 = joystick.a();
    private final Trigger requestL2 = joystick.b();
    private final Trigger requestL3 = joystick.x();
    private final Trigger requestL4 = joystick.y();


    private final Trigger zeroElevator = joystick.start();
    private final Trigger algaeArmZero = joystick.back();


    private final Trigger highAlgaePickup = joystick.leftBumper();
    private final Trigger lowAlgaePickup = joystick.rightBumper();
    private final Trigger leftBranchScore = joystick.leftTrigger();
    private final Trigger rightBranchScore = joystick.rightTrigger();

    public CompBotControls(Joystick driveStick, Joystick steerStick, CommandXboxController joystick) {
        super(driveStick, steerStick, joystick);
    }

    @Override
    public void bindControls() {
        requestL1.onTrue(Commands.print("Request L1"));
        requestL2.onTrue(Commands.print("Request L2"));
        requestL3.onTrue(Commands.print("Request L3"));
        requestL4.onTrue(Commands.print("Request L4"));

        bindSmartDashboardCommands();
    }

    private void bindSmartDashboardCommands() {
        SmartDashboard.putData("Zero Yaw", new ZeroYawCommand());
        SmartDashboard.putData("Set LLs to Apriltag", new PipelineSwitcher(Pipeline.April));
        SmartDashboard.putData("Set LLs to Viewfinder", new PipelineSwitcher(Pipeline.View));
    }
}
