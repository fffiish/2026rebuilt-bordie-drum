package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.HashMap;

public class VisionConstants {
  // spotless:off
    public static final HashMap<Integer, Pose3d> kWeldedAprilTagField = new HashMap<>();
    public static final HashMap<Integer, Pose3d> kAndyMarkAprilTagField = new HashMap<>();
    public static final HashMap<Integer, Pose3d> kAprilTagField = kWeldedAprilTagField;
    static {
        kWeldedAprilTagField.put(1, new Pose3d(new Translation3d(16.697198, 0.655320, 1.485900), new Rotation3d(new Quaternion(0.453990, 0.000000, 0.000000, 0.891007))));
        kWeldedAprilTagField.put(2, new Pose3d(new Translation3d(16.697198, 7.396480, 1.485900), new Rotation3d(new Quaternion(-0.453990, -0.000000, 0.000000, 0.891007))));
        kWeldedAprilTagField.put(3, new Pose3d(new Translation3d(11.560810, 8.055610, 1.301750), new Rotation3d(new Quaternion(-0.707107, -0.000000, 0.000000, 0.707107))));
        kWeldedAprilTagField.put(4, new Pose3d(new Translation3d(9.276080, 6.137656, 1.867916), new Rotation3d(new Quaternion(0.965926, 0.000000, 0.258819, 0.000000))));
        kWeldedAprilTagField.put(5, new Pose3d(new Translation3d(9.276080, 1.914906, 1.867916), new Rotation3d(new Quaternion(0.965926, 0.000000, 0.258819, 0.000000))));
        kWeldedAprilTagField.put(6, new Pose3d(new Translation3d(13.474446, 3.306318, 0.308102), new Rotation3d(new Quaternion(-0.866025, -0.000000, 0.000000, 0.500000))));
        kWeldedAprilTagField.put(7, new Pose3d(new Translation3d(13.890498, 4.025900, 0.308102), new Rotation3d(new Quaternion(1.000000, 0.000000, 0.000000, 0.000000))));
        kWeldedAprilTagField.put(8, new Pose3d(new Translation3d(13.474446, 4.745482, 0.308102), new Rotation3d(new Quaternion(0.866025, 0.000000, 0.000000, 0.500000))));
        kWeldedAprilTagField.put(9, new Pose3d(new Translation3d(12.643358, 4.745482, 0.308102), new Rotation3d(new Quaternion(0.500000, 0.000000, 0.000000, 0.866025))));
        kWeldedAprilTagField.put(10, new Pose3d(new Translation3d(12.227306, 4.025900, 0.308102), new Rotation3d(new Quaternion(0.000000, 0.000000, 0.000000, 1.000000))));
        kWeldedAprilTagField.put(11, new Pose3d(new Translation3d(12.643358, 3.306318, 0.308102), new Rotation3d(new Quaternion(-0.500000, -0.000000, 0.000000, 0.866025))));
        kWeldedAprilTagField.put(12, new Pose3d(new Translation3d(0.851154, 0.655320, 1.485900), new Rotation3d(new Quaternion(0.891007, 0.000000, 0.000000, 0.453990))));
        kWeldedAprilTagField.put(13, new Pose3d(new Translation3d(0.851154, 7.396480, 1.485900), new Rotation3d(new Quaternion(-0.891007, -0.000000, 0.000000, 0.453990))));
        kWeldedAprilTagField.put(14, new Pose3d(new Translation3d(8.272272, 6.137656, 1.867916), new Rotation3d(new Quaternion(0.000000, -0.258819, 0.000000, 0.965926))));
        kWeldedAprilTagField.put(15, new Pose3d(new Translation3d(8.272272, 1.914906, 1.867916), new Rotation3d(new Quaternion(0.000000, -0.258819, 0.000000, 0.965926))));
        kWeldedAprilTagField.put(16, new Pose3d(new Translation3d(5.987542, -0.003810, 1.301750), new Rotation3d(new Quaternion(0.707107, 0.000000, 0.000000, 0.707107))));
        kWeldedAprilTagField.put(17, new Pose3d(new Translation3d(4.073906, 3.306318, 0.308102), new Rotation3d(new Quaternion(-0.500000, -0.000000, 0.000000, 0.866025))));
        kWeldedAprilTagField.put(18, new Pose3d(new Translation3d(3.657600, 4.025900, 0.308102), new Rotation3d(new Quaternion(0.000000, 0.000000, 0.000000, 1.000000))));
        kWeldedAprilTagField.put(19, new Pose3d(new Translation3d(4.073906, 4.745482, 0.308102), new Rotation3d(new Quaternion(0.500000, 0.000000, 0.000000, 0.866025))));
        kWeldedAprilTagField.put(20, new Pose3d(new Translation3d(4.904740, 4.745482, 0.308102), new Rotation3d(new Quaternion(0.866025, 0.000000, 0.000000, 0.500000))));
        kWeldedAprilTagField.put(21, new Pose3d(new Translation3d(5.321046, 4.025900, 0.308102), new Rotation3d(new Quaternion(1.000000, 0.000000, 0.000000, 0.000000))));
        kWeldedAprilTagField.put(22, new Pose3d(new Translation3d(4.904740, 3.306318, 0.308102), new Rotation3d(new Quaternion(-0.866025, -0.000000, 0.000000, 0.500000))));

        kAndyMarkAprilTagField.put(1, new Pose3d(new Translation3d(16.687292, 0.628142, 1.485900), new Rotation3d(new Quaternion(0.453990, 0.000000, 0.000000, 0.891007))));
        kAndyMarkAprilTagField.put(2, new Pose3d(new Translation3d(16.687292, 7.414260, 1.485900), new Rotation3d(new Quaternion(-0.453990, -0.000000, 0.000000, 0.891007))));
        kAndyMarkAprilTagField.put(3, new Pose3d(new Translation3d(11.490960, 8.031734, 1.301750), new Rotation3d(new Quaternion(-0.707107, -0.000000, 0.000000, 0.707107))));
        kAndyMarkAprilTagField.put(4, new Pose3d(new Translation3d(9.276080, 6.132576, 1.867916), new Rotation3d(new Quaternion(0.965926, 0.000000, 0.258819, 0.000000))));
        kAndyMarkAprilTagField.put(5, new Pose3d(new Translation3d(9.276080, 1.909826, 1.867916), new Rotation3d(new Quaternion(0.965926, 0.000000, 0.258819, 0.000000))));
        kAndyMarkAprilTagField.put(6, new Pose3d(new Translation3d(13.474446, 3.301238, 0.308102), new Rotation3d(new Quaternion(-0.866025, -0.000000, 0.000000, 0.500000))));
        kAndyMarkAprilTagField.put(7, new Pose3d(new Translation3d(13.890498, 4.020820, 0.308102), new Rotation3d(new Quaternion(1.000000, 0.000000, 0.000000, 0.000000))));
        kAndyMarkAprilTagField.put(8, new Pose3d(new Translation3d(13.474446, 4.740402, 0.308102), new Rotation3d(new Quaternion(0.866025, 0.000000, 0.000000, 0.500000))));
        kAndyMarkAprilTagField.put(9, new Pose3d(new Translation3d(12.643358, 4.740402, 0.308102), new Rotation3d(new Quaternion(0.500000, 0.000000, 0.000000, 0.866025))));
        kAndyMarkAprilTagField.put(10, new Pose3d(new Translation3d(12.227306, 4.020820, 0.308102), new Rotation3d(new Quaternion(0.000000, 0.000000, 0.000000, 1.000000))));
        kAndyMarkAprilTagField.put(11, new Pose3d(new Translation3d(12.643358, 3.301238, 0.308102), new Rotation3d(new Quaternion(-0.500000, -0.000000, 0.000000, 0.866025))));
        kAndyMarkAprilTagField.put(12, new Pose3d(new Translation3d(0.861314, 0.628142, 1.485900), new Rotation3d(new Quaternion(0.891007, 0.000000, 0.000000, 0.453990))));
        kAndyMarkAprilTagField.put(13, new Pose3d(new Translation3d(0.861314, 7.414260, 1.485900), new Rotation3d(new Quaternion(-0.891007, -0.000000, 0.000000, 0.453990))));
        kAndyMarkAprilTagField.put(14, new Pose3d(new Translation3d(8.272272, 6.132576, 1.867916), new Rotation3d(new Quaternion(0.000000, -0.258819, 0.000000, 0.965926))));
        kAndyMarkAprilTagField.put(15, new Pose3d(new Translation3d(8.272272, 1.909826, 1.867916), new Rotation3d(new Quaternion(0.000000, -0.258819, 0.000000, 0.965926))));
        kAndyMarkAprilTagField.put(16, new Pose3d(new Translation3d(6.057646, 0.010668, 1.301750), new Rotation3d(new Quaternion(0.707107, 0.000000, 0.000000, 0.707107))));
        kAndyMarkAprilTagField.put(17, new Pose3d(new Translation3d(4.073906, 3.301238, 0.308102), new Rotation3d(new Quaternion(-0.500000, -0.000000, 0.000000, 0.866025))));
        kAndyMarkAprilTagField.put(18, new Pose3d(new Translation3d(3.657600, 4.020820, 0.308102), new Rotation3d(new Quaternion(0.000000, 0.000000, 0.000000, 1.000000))));
        kAndyMarkAprilTagField.put(19, new Pose3d(new Translation3d(4.073906, 4.740402, 0.308102), new Rotation3d(new Quaternion(0.500000, 0.000000, 0.000000, 0.866025))));
        kAndyMarkAprilTagField.put(20, new Pose3d(new Translation3d(4.904740, 4.740402, 0.308102), new Rotation3d(new Quaternion(0.866025, 0.000000, 0.000000, 0.500000))));
        kAndyMarkAprilTagField.put(21, new Pose3d(new Translation3d(5.321046, 4.020820, 0.308102), new Rotation3d(new Quaternion(1.000000, 0.000000, 0.000000, 0.000000))));
        kAndyMarkAprilTagField.put(22, new Pose3d(new Translation3d(4.904740, 3.301238, 0.308102), new Rotation3d(new Quaternion(-0.866025, -0.000000, 0.000000, 0.500000))));
    }
    // spotless:on
}
