import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;

import org.junit.jupiter.api.Test;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PPFlippingUtilTest {
    @Test
    public void testFlip() {
        Pose2d rr_pose = new Pose2d(12.275, 5.4, Rotation2d.fromDegrees(-60));
        Pose2d br_pose = FlippingUtil.flipFieldPose(rr_pose);


        Pose2d expected = new Pose2d(5.27, 2.65, Rotation2d.fromDegrees(120));
        System.out.println(br_pose);
        System.out.println(expected);
        
        assertEquals(expected.getTranslation(), br_pose.getTranslation());
        assertEquals(expected.getRotation(), br_pose.getRotation());
        assertEquals(expected, br_pose);
        fail();
    }
}
