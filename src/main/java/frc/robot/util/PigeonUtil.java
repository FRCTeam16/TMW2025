package frc.robot.util;

import com.ctre.phoenix6.hardware.Pigeon2;

public class PigeonUtil {
    private static final Pigeon2 pigeon = new Pigeon2(1); // TODO: have not magic number here

    public static double getHeading(){
        return pigeon.getYaw().getValueAsDouble();
    }

}
