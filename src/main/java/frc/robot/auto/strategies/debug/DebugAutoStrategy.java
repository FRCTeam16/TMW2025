package frc.robot.auto.strategies.debug;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.DriveRobotCentricCommand;
import frc.robot.commands.RotateToAngleCommand;
import frc.robot.commands.vision.AlignDriveInCommand;
import frc.robot.commands.vision.SimpleAlignCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorSetpoint;

import static edu.wpi.first.units.Units.*;

public class DebugAutoStrategy extends SequentialCommandGroup {
    public DebugAutoStrategy() {
        this.addCommands(
                new PrintCommand("Debug Auto Strategy")
        );
    }

    public Command testingFirstDrive() {
        return Commands.sequence(
                new DriveRobotCentricCommand(Seconds.of(1.3)),
                new RotateToAngleCommand(Degrees.of(120)).withTimeout(0.5),
//                Commands.runOnce(() -> {
//                    Subsystems.swerveSubsystem.setControl(new SwerveRequest.SwerveDriveBrake());
//                }),
                new AlignDriveInCommand(AlignDriveInCommand.AlignTarget.RIGHT),
                doScoreSequence()
        );
    }


    public Command doScoreSequence() {
        return Commands.sequence(
                new Elevator.ElevatorMoveToPositionCommand(Elevator.ElevatorSetpoint.L4).withTimeout(1.5),
                Subsystems.coralIntake.shootCoralCommand().withTimeout(0.5),
                Commands.parallel(
                        Subsystems.coralIntake.stopCommand(),
                        new Elevator.ElevatorMoveToPositionCommand(ElevatorSetpoint.Zero).withNoWait(),
                        new WaitCommand(0.6)
                )
        ).withName("Scoring Sequence");
    }

}
