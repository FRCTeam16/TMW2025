package frc.robot.commands.dms.MotorTestDMS;

import frc.robot.subsystems.DMS.SwerveDataCollector;

public class MotorTest2 extends AbstractRunMotorTest<SwerveDataCollector> {

    public MotorTest2(SwerveDataCollector dmsDataCollector) {
        super(dmsDataCollector);
    }

    @Override
    protected void startMotor() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'startMotor'");
    }

    @Override
    protected void stopMotor() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stopMotor'");
    }

    @Override
    protected double[] getMotorCurrents() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMotorCurrents'");
    }

    
}





/*  what to do
use dms files to understand 

create a command for each subsytem 

run all of them dms data collector

create a runDMSCommand to use all of them in a chain

then use a timer for each command like this in runDMS
new WaitCommand(1.0),

use types of filter seen here
https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/introduction.html
median for 


ways to do this
create a new file for every motor? seems clunky
add all to one file
*/