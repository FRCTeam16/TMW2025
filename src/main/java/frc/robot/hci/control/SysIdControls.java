package frc.robot.hci.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Subsystems;
import frc.robot.subsystems.SysIdHelper.Routine;

public class SysIdControls extends ControlBinding {
    public SysIdControls(Joystick driveStick, Joystick steerStick, CommandXboxController joystick) {
        super(driveStick, steerStick, joystick);
    }

    @Override
    public void bindControls() {
        // Run SysId routines when holding back/start and X/Y. // these are fine for now
        // View that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(Subsystems.swerveSubsystem.getSysIdHelper().withRoutineToApply(Routine.Translation).sysIdDynamic(SysIdRoutine.Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(Subsystems.swerveSubsystem.getSysIdHelper().withRoutineToApply(Routine.Translation).sysIdDynamic(SysIdRoutine.Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(Subsystems.swerveSubsystem.getSysIdHelper().withRoutineToApply(Routine.Translation).sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(Subsystems.swerveSubsystem.getSysIdHelper().withRoutineToApply(Routine.Translation).sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        joystick.leftTrigger().and(joystick.y()).whileTrue(Subsystems.swerveSubsystem.getSysIdHelper().withRoutineToApply(Routine.Steer).sysIdDynamic(SysIdRoutine.Direction.kForward));
        joystick.leftTrigger().and(joystick.x()).whileTrue(Subsystems.swerveSubsystem.getSysIdHelper().withRoutineToApply(Routine.Steer).sysIdDynamic(SysIdRoutine.Direction.kReverse));
        joystick.leftTrigger().and(joystick.a()).whileTrue(Subsystems.swerveSubsystem.getSysIdHelper().withRoutineToApply(Routine.Steer).sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        joystick.leftTrigger().and(joystick.b()).whileTrue(Subsystems.swerveSubsystem.getSysIdHelper().withRoutineToApply(Routine.Steer).sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        joystick.rightTrigger().and(joystick.y()).whileTrue(Subsystems.swerveSubsystem.getSysIdHelper().withRoutineToApply(Routine.Rotation).sysIdDynamic(SysIdRoutine.Direction.kForward));
        joystick.rightTrigger().and(joystick.x()).whileTrue(Subsystems.swerveSubsystem.getSysIdHelper().withRoutineToApply(Routine.Rotation).sysIdDynamic(SysIdRoutine.Direction.kReverse));
        joystick.rightTrigger().and(joystick.a()).whileTrue(Subsystems.swerveSubsystem.getSysIdHelper().withRoutineToApply(Routine.Rotation).sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        joystick.rightTrigger().and(joystick.b()).whileTrue(Subsystems.swerveSubsystem.getSysIdHelper().withRoutineToApply(Routine.Rotation).sysIdQuasistatic(SysIdRoutine.Direction.kReverse));



        // DMS
//        joystick.a().onTrue(new RunDMSCommand());
    }
}
