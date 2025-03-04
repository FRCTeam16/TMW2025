package frc.robot.hci.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.commands.pose.ZeroYawCommand;
import frc.robot.subsystems.scoring.AlgaePickRequest;
import frc.robot.subsystems.scoring.AutoScoreCoralCommand;
import frc.robot.subsystems.scoring.CoralScoringRequest;
import frc.robot.subsystems.scoring.ScoringGoals;
import frc.robot.commands.vision.PipelineSwitcher;
import frc.robot.subsystems.vision.Pipeline;

import java.util.function.Supplier;

public class CompBotControls extends ControlBinding {

    //
    // Driver Controls
    //
    private final Trigger doAlgaePick = new Trigger(steerStick::getTrigger);
    private final Trigger alignWithBarge = new JoystickButton(steerStick, 2);

    private final Trigger doScore = new Trigger(driveStick::getTrigger);    // if in barge or processor zone score algae, otherwise score coral
    private final Trigger alignWithProcessor = new JoystickButton(driveStick, 2);
    private final Trigger alignWithLeftStation = new JoystickButton(driveStick, 3);
    private final Trigger alignWithRightStation = new JoystickButton(driveStick, 4);

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

    private void bindDriverControls() {
        doAlgaePick.whileTrue(Subsystems.scoreSubsystem.pickAlgaeCmd());
        alignWithBarge.whileTrue(Commands.print("Auto Align with Barge"));

        doScore.whileTrue(new AutoScoreCoralCommand());
        alignWithProcessor.whileTrue(Commands.print("Auto Align with Processor"));
        alignWithLeftStation.whileTrue(Commands.print("Auto Align with Left Station"));
        alignWithRightStation.whileTrue(Commands.print("Auto Align with Right Station"));

    }

    private void bindOperatorControls() {
        requestL1.onTrue(Subsystems.scoreSubsystem.requestCoralScoreCmd(new CoralScoringRequest().withReefLevel(ScoringGoals.CoralGoals.ReefLevels.L1)));
        requestL2.onTrue(Subsystems.scoreSubsystem.requestCoralScoreCmd(new CoralScoringRequest().withReefLevel(ScoringGoals.CoralGoals.ReefLevels.L2)));
        requestL3.onTrue(Subsystems.scoreSubsystem.requestCoralScoreCmd(new CoralScoringRequest().withReefLevel(ScoringGoals.CoralGoals.ReefLevels.L3)));
        requestL4.onTrue(Subsystems.scoreSubsystem.requestCoralScoreCmd(new CoralScoringRequest().withReefLevel(ScoringGoals.CoralGoals.ReefLevels.L4)));
        requestLeftBranchScore.onTrue(Subsystems.scoreSubsystem.requestCoralScoreCmd(new CoralScoringRequest().withBranchScore(ScoringGoals.CoralGoals.BranchScore.LEFT)));
        requestRightBranchScore.onTrue(Subsystems.scoreSubsystem.requestCoralScoreCmd(new CoralScoringRequest().withBranchScore(ScoringGoals.CoralGoals.BranchScore.RIGHT)));

        requestHighAlgaePick.onTrue(Subsystems.scoreSubsystem.requestAlgaeScoreCmd(new AlgaePickRequest().withAlgaePick(ScoringGoals.AlgaeGoals.AlgaePick.HIGH)));
        requestLowAlgaePick.onTrue(Subsystems.scoreSubsystem.requestAlgaeScoreCmd(new AlgaePickRequest().withAlgaePick(ScoringGoals.AlgaeGoals.AlgaePick.LOW)));

        manualAlgaeToggleButton.toggleOnTrue(Subsystems.algaeArm.openLoopCommand(manualAlgaeArmControl));
        manualElevatorToggleButton.toggleOnTrue(Subsystems.elevator.openLoopCommand(manualElevatorControl));

        zeroElevator.onTrue(Commands.print("Zero Elevator"));
        zeroAlgaeArm.onTrue(Commands.print("Zero Algae Arm"));
    }

    private void bindSmartDashboardCommands() {
        SmartDashboard.putData("Zero Yaw", new ZeroYawCommand());
        SmartDashboard.putData("Set LLs to Apriltag", new PipelineSwitcher(Pipeline.April));
        SmartDashboard.putData("Set LLs to Viewfinder", new PipelineSwitcher(Pipeline.View));
    }
}
