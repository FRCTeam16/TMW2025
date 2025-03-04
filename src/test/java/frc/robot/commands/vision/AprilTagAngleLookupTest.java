package frc.robot.commands.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.vision.AprilTagAngleLookup;
import org.junit.jupiter.api.Test;

import java.util.Optional;

import static edu.wpi.first.units.Units.Degree;
import static org.junit.jupiter.api.Assertions.*;

public class AprilTagAngleLookupTest {
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);   // US regionals use welded


    @Test
    public void checkAll() {
        fieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
        for (int i=0;i<23;i++) {
            Optional<Pose3d> pose3d = fieldLayout.getTagPose(i);
            if (pose3d.isPresent()) {
                double degrees = pose3d.get().toPose2d().getRotation().getDegrees();
                System.out.println("TAG: " + i + " | Degrees: " + degrees + " + | Lookup: " + AprilTagAngleLookup.getFacingAngle(i).get());
            }
        }
    }

    @Test
    public void testTransform() {
        Pose2d tag = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(60));
        Transform2d transform2d = new Transform2d(-0.5, -.25, Rotation2d.fromDegrees(0));
        Pose2d baseresult = tag.transformBy(transform2d);
        Pose2d result = new Pose2d(baseresult.getTranslation(), Rotation2d.fromDegrees(300));
        System.out.println(tag);
        System.out.println(baseresult);
        System.out.println(result);
    }

//    @Test
    public void testGetFacingAngle() {
        Optional<Angle> angle = AprilTagAngleLookup.getFacingAngle(17);
        assertTrue(angle.isPresent());
        angle.get().in(Units.Degrees);
        assertEquals(45.0, angle.get().in(Units.Degrees), 0.01);


        Optional<Pose3d> fieldPose = fieldLayout.getTagPose(17);
        assertTrue(fieldPose.isPresent());
        double degrees = fieldPose.get().toPose2d().getRotation().getDegrees();
        assertEquals(angle.get().in(Degree), degrees, 0.01);

        angle = AprilTagAngleLookup.getFacingAngle(99);
        assertFalse(angle.isPresent());
    }

}
