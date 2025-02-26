package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;

public class TranslationController extends PIDController {
    public TranslationController(double kp, double ki, double kd) {
        super(kp, ki, kd);
    }

    public TranslationController() {
        super(0, 0, 0);
    }
}
