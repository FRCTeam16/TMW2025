package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.LinearFilter;

public class MotorVoltageFilter {
    private LinearFilter filter = LinearFilter.movingAverage(5); // taps == Num of values
    private TalonFX motor;
    private double threshold;
    private double current = 0;

    public MotorVoltageFilter(double Threshold, TalonFX Motor){
        motor = Motor;
        threshold = Threshold;
    }

    public void update(){
        current = filter.calculate(motor.getMotorVoltage().getValueAsDouble());
    }

    public boolean isOverThreshold(){
        return current > threshold;
    }
    
}
