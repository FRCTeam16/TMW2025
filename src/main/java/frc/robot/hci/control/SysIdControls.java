package frc.robot.hci.control;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Subsystems;
import frc.robot.commands.dms.RunDMSCommand;

public class SysIdControls extends ControlBinding {
    public SysIdControls(Joystick driveStick, Joystick steerStick, CommandXboxController joystick) {
        super(driveStick, steerStick, joystick);
    }

    @Override
    public void bindControls() {
        // Run SysId routines when holding back/start and X/Y. // these are fine for now
        // View that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(Subsystems.swerveSubsystem.getSysIdHelper().sysIdDynamic(SysIdRoutine.Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(Subsystems.swerveSubsystem.getSysIdHelper().sysIdDynamic(SysIdRoutine.Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(Subsystems.swerveSubsystem.getSysIdHelper().sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(Subsystems.swerveSubsystem.getSysIdHelper().sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        // DMS
        joystick.a().onTrue(new RunDMSCommand());
    }
}
