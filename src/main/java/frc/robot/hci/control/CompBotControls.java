package frc.robot.hci.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.commands.ZeroYawCommand;
import frc.robot.commands.vision.PipelineSwitcher;
import frc.robot.subsystems.vision.Pipeline;

import java.util.function.Supplier;

public class CompBotControls extends ControlBinding {

    //
    // Operator Controls
    //

    private final Trigger requestL1 = joystick.a();
    private final Trigger requestL2 = joystick.b();
    private final Trigger requestL3 = joystick.x();
    private final Trigger requestL4 = joystick.y();

    private final Trigger requestHighAlgaePick = joystick.leftBumper();
    private final Trigger requestLowAlgaePick = joystick.rightBumper();
    private final Trigger requestLeftBranchScore = joystick.leftTrigger();
    private final Trigger requestRightBranchScore = Robot.isReal() ?
            joystick.rightTrigger() :
            thresholdTrigger(joystick::getRightY, 0.5);  // simulation mode is flipped

    private final Trigger zeroElevator = joystick.start();
    private final Trigger zeroAlgaeArm = joystick.back();

    private final Trigger manualAlgaeToggleButton = joystick.leftStick();
    private final Supplier<Double> manualAlgaeArmControl = deadband(joystick::getLeftY, 0.05);
    private final Trigger manualElevatorToggleButton = joystick.rightStick();
    private final Supplier<Double> manualElevatorControl = Robot.isReal() ?
            deadband(joystick::getRightY, 0.05) :
            deadband(joystick::getRightTriggerAxis, 0.05);  // simulation mode is flipped


    public CompBotControls(Joystick driveStick, Joystick steerStick, CommandXboxController joystick) {
        super(driveStick, steerStick, joystick);
    }

    @Override
    public void bindControls() {
        bindOperatorControls();
        bindDriverControls();
        bindSmartDashboardCommands();
    }

    private void bindOperatorControls() {
        requestL1.onTrue(Commands.print("Request L1"));
        requestL2.onTrue(Commands.print("Request L2"));
        requestL3.onTrue(Commands.print("Request L3"));
        requestL4.onTrue(Commands.print("Request L4"));

        requestHighAlgaePick.onTrue(Commands.print("Request High Algae Pick"));
        requestLowAlgaePick.onTrue(Commands.print("Request Low Algae Pick"));
        requestLeftBranchScore.onTrue(Commands.print("Request Left Branch Score"));
        requestRightBranchScore.onTrue(Commands.print("Request Right Branch Score"));

        manualAlgaeToggleButton.toggleOnTrue(Subsystems.algaeArm.openLoopCommand(manualAlgaeArmControl));
        manualElevatorToggleButton.toggleOnTrue(Subsystems.elevator.openLoopCommand(manualElevatorControl));

        zeroElevator.onTrue(Commands.print("Zero Elevator"));
        zeroAlgaeArm.onTrue(Commands.print("Zero Algae Arm"));
    }

    private void bindDriverControls() {
    }

    private void bindSmartDashboardCommands() {
        SmartDashboard.putData("Zero Yaw", new ZeroYawCommand());
        SmartDashboard.putData("Set LLs to Apriltag", new PipelineSwitcher(Pipeline.April));
        SmartDashboard.putData("Set LLs to Viewfinder", new PipelineSwitcher(Pipeline.View));
    }
}
