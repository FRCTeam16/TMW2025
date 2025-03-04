package frc.robot.subsystems.DMS;

public class CoralDataCollector extends AbstractDataCollector<CoralDataCollector.CoralData> {

    @Override
    public CoralData getCollectedData() {
        return null;
    }

    public static class CoralData {
        public double[] getMotorCurrents() {
            return null;
        }

        public double[] getMotorVelocities() {
            return null;
        }
    }
}
