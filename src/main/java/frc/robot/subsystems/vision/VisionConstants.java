// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.lib.LoggedTunableNumber;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "limelight-zero";
  public static String camera1Name = "limelight-one";
  public static String camera2Name = "limelight-two";
  public static String camera3Name = "limelight-three";
  public static String turretCameraName = "limelight-turret";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  // Forward, left, up (so just multiply LL right by -1)
  public static Transform3d robotToCamera0 =
      new Transform3d(
          -0.28, -0.309, 0.223, new Rotation3d(0.0, Math.toRadians(18.6), Math.toRadians(-65)));
  public static Transform3d robotToCamera1 =
      new Transform3d(
          -0.28, 0.309, 0.223, new Rotation3d(0.0, Math.toRadians(18.6), Math.toRadians(65)));
  public static Transform3d robotToCamera2 =
      new Transform3d(
          -0.299, 0.3, 0.223, new Rotation3d(0.0, Math.toRadians(17.8), Math.toRadians(210)));
  public static Transform3d robotToCamera3 =
      new Transform3d(
          -0.299, -0.3, 0.223, new Rotation3d(0.0, Math.toRadians(17.8), Math.toRadians(150)));
  ;

  // Basic filtering thresholds
  public static LoggedTunableNumber maxAmbiguity =
      new LoggedTunableNumber("Vision/MaxAmbiguity", 0.3);
  public static LoggedTunableNumber maxZError = new LoggedTunableNumber("Vision/MaxZError", 0.75);

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static LoggedTunableNumber linearStdDevBaseline =
      new LoggedTunableNumber("Vision/LinearStdDevBaseline", 0.35 * 2); // Meters
  public static LoggedTunableNumber angularStdDevBaseline =
      new LoggedTunableNumber("Vision/AngularStdDevBaseline", 0.36 * 2); // Radians

  // Multipliers to apply for MegaTag 2 observations
  public static LoggedTunableNumber linearStdDevMegatag2Factor =
      new LoggedTunableNumber(
          "Vision/LinearStdDevMegatag2Factor", 0.25); // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  // Various tag stdev multipliers
  public static double getTagStdevMultiplier(int tag) {
    switch (tag) {
      case 9, 10, 11, 2, 8, 5, 4, 3, 19, 20, 21, 24, 18, 27, 26, 25: // HUB TAGS
        return 1.0;
      case 14, 13, 15, 16, 29, 30, 31, 32: // OUTPOST, TOWER TAGS
        return 3.5;
      case 1, 6, 22, 17: // TRENCH TAGS SEEN FROM NEUTRAL ZONE
        return 1.0;
      case 12, 7, 28, 23:
        return 9.0; // TRENCH TAGS SEEN FROM ALLIANCE ZONE
      default:
        return Double.POSITIVE_INFINITY; // Unknown tag, reject
    }
  }
}
