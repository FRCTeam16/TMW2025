package frc.robot.commands.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.vision.AprilTagAngleLookup;
import org.junit.jupiter.api.Test;

import java.util.Optional;

import static edu.wpi.first.units.Units.Degree;
import static org.junit.jupiter.api.Assertions.*;

public class AprilTagAngleLookupTest {

    @Test
    public void testGetFacingAngle() {
        Optional<Angle> angle = AprilTagAngleLookup.getFacingAngle(17);
        assertTrue(angle.isPresent());
        angle.get().in(Units.Degrees);
        assertEquals(45.0, angle.get().in(Units.Degrees), 0.01);

        Optional<Angle> fieldAngle = AprilTagAngleLookup.getFacingAngle(17);
        assertTrue(fieldAngle.isPresent());
        assertEquals(angle.get().in(Degree), fieldAngle.get().in(Units.Degrees), 0.01);

        angle = AprilTagAngleLookup.getFacingAngle(99);
        assertFalse(angle.isPresent());
    }

}
