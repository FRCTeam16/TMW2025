package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.LinearFilter;

public class MotorStatorCurrentFilter {
    private LinearFilter filter = LinearFilter.movingAverage(5); // taps == Num of values
    private TalonFX motor;
    private double threshold;
    private double current = 0;

    public MotorStatorCurrentFilter(double Threshold, TalonFX Motor){
        motor = Motor;
        threshold = Threshold;
    }

    public void update(){
        current = filter.calculate(motor.getStatorCurrent().getValueAsDouble());
    }

    public boolean isOverThreshold(){
        return current > threshold;
    }
    
}
