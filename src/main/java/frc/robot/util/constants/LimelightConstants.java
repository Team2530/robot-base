package frc.robot.util.constants;

import java.util.HashMap;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

public final class LimelightConstants {
    private static final AprilTagFieldLayout TAG_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public final static HashMap<Integer, Pose2d> TAG_POSES =
        new HashMap<Integer, Pose2d>() {{
            for (int i = 0; i < TAG_LAYOUT.getTags().size(); ++i) {
                if (TAG_LAYOUT.getTagPose(i + 1).isPresent())
                    put(i, TAG_LAYOUT.getTagPose(i + 1).get().toPose2d());
            }
        }};

    public static final int[] VALID_TAGS = {
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
        19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34,
        35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48
    };
}
