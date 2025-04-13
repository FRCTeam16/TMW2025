package frc.robot.util;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Current;

// FIXME: Rename
public class MotorStatorCurrentFilter {
    // private LinearFilter filter = LinearFilter.movingAverage(5); // taps == Num of values
    private final LinearFilter filter = LinearFilter.highPass(0.1, 0.02);
    private final Current threshold;
    private Current current = Amps.of(0);
    private final Supplier<Current> currentSupplier;
    
        public MotorStatorCurrentFilter(Current threshold, Supplier<Current> currentSupplier){
            this.threshold = threshold;
            this.currentSupplier = currentSupplier;
    }

    public Current update() {
        double absAmps = currentSupplier.get().in(Amps);
        current = Amp.of(filter.calculate(absAmps));
        return current;
    }

    public Current getCurrent(){
        return current;
    }

    public boolean isOverThreshold(){
        return current.gt(threshold);
    }

    public boolean isUnderThreshold() {
        return current.lt(threshold);
    }
    
}
