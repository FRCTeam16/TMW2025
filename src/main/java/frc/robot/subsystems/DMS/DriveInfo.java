package frc.robot.subsystems.DMS;

/** Add your docs here. */
public class DriveInfo <T> {
public T FL;
public T FR;
public T RL;
public T RR;

public DriveInfo(T value){
    FL = value;
    FR = value;
    RL = value;
    RR = value;
    }

    public DriveInfo(T fl, T fr, T rl, T rr){
        FL = fl;
        FR = fr;
        RL = rl;
        RR = rr;
        }

}


