// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    // Add sim camera (Limelight 4, in reality we do 1280x800 but it lowk performs like 640x400)
    var cameraProperties = new SimCameraProperties();
    double cx = 320.0 / 2.0 - 0.5;
    double cy = 200.0 / 2.0 - 0.5;
    double fx = cx / Math.tan(Math.toRadians(82.0 / 2.0));
    double fy = cy / Math.tan(Math.toRadians(56.2 / 2.0));
    cameraProperties.setCalibration(
        320,
        200,
        MatBuilder.fill(Nat.N3(), Nat.N3(), fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0),
        VecBuilder.fill(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    cameraSim = new PhotonCameraSim(camera, cameraProperties, aprilTagLayout);
    visionSim.addCamera(cameraSim, fixRobotToCamera(robotToCamera));
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }

  @Override
  public void setRobotOffset(Transform3d offset) {
    visionSim.adjustCamera(cameraSim, fixRobotToCamera(offset));
    super.setRobotOffset(offset);
  }
}
