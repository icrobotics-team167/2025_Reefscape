// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.cotc.Robot;
import frc.cotc.vision.FiducialPoseEstimatorIO.FiducialPoseEstimatorIOInputs.FiducialPoseEstimate;
import frc.cotc.vision.FiducialPoseEstimatorIO.FiducialPoseEstimatorIOInputs.FiducialPoseEstimate.AprilTag;
import java.util.ArrayList;
import java.util.HashMap;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class FiducialPoseEstimatorIOPhoton implements FiducialPoseEstimatorIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
  private final FiducialPoseEstimatorIOConstantsAutoLogged constants;

  public FiducialPoseEstimatorIOPhoton(String name, Transform3d robotToCameraTransform) {
    camera = new PhotonCamera(name);
    poseEstimator =
        new PhotonPoseEstimator(
            FiducialPoseEstimator.tagLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCameraTransform);

    constants = new FiducialPoseEstimatorIOConstantsAutoLogged();
    constants.robotToCameraTransform = robotToCameraTransform;

    if (Robot.isSimulation()) {
      Sim.addCamera(camera, robotToCameraTransform);
    }
  }

  private final ArrayList<FiducialPoseEstimate> estimatesList = new ArrayList<>();

  @Override
  public void updateInputs(FiducialPoseEstimatorIOInputs inputs) {
    var results = camera.getAllUnreadResults();

    for (var result : results) {
      poseEstimator
          .update(result)
          .ifPresent(
              estimate -> {
                var tagsUsed = new AprilTag[estimate.targetsUsed.size()];
                for (int i = 0; i < tagsUsed.length; i++) {
                  var tag = estimate.targetsUsed.get(i);
                  tagsUsed[i] =
                      new AprilTag(
                          estimate
                              .estimatedPose
                              .plus(constants.robotToCameraTransform)
                              .plus(tag.bestCameraToTarget),
                          tag.fiducialId,
                          tag.bestCameraToTarget.getTranslation().getNorm(),
                          Units.degreesToRadians(-tag.pitch), // Photon is up+, we need down+
                          Units.degreesToRadians(-tag.yaw), // Photon is right+, we need left+
                          tag.poseAmbiguity);
                }
                estimatesList.add(
                    new FiducialPoseEstimate(
                        estimate.estimatedPose, estimate.timestampSeconds, tagsUsed));
              });
    }

    inputs.poseEstimates = estimatesList.toArray(new FiducialPoseEstimate[0]);
    estimatesList.clear();
  }

  @Override
  public FiducialPoseEstimatorIOConstantsAutoLogged getConstants() {
    return constants;
  }

  public static class Sim {
    private static VisionSystemSim visionSystemSim;

    static void addCamera(PhotonCamera camera, Transform3d transform) {
      if (visionSystemSim == null) {
        visionSystemSim = new VisionSystemSim("main");
        visionSystemSim.addAprilTags(FiducialPoseEstimator.tagLayout);
      }

      var name = camera.getName();
      SimCameraProperties properties;
      if (propertiesHashMap.containsKey(name)) {
        properties = propertiesHashMap.get(name);
      } else {
        properties = propertiesHashMap.get("None");
      }

      var cameraSim = new PhotonCameraSim(camera, properties);
      cameraSim.enableProcessedStream(false);
      cameraSim.enableRawStream(false);
      cameraSim.enableDrawWireframe(false);
      visionSystemSim.addCamera(cameraSim, transform);
    }

    public static void update() {
      visionSystemSim.update(Robot.groundTruthPoseSupplier.get());
    }

    private static final HashMap<String, SimCameraProperties> propertiesHashMap = new HashMap<>();

    static {
      var none = new SimCameraProperties();
      none.setCalibration(1280, 800, Rotation2d.fromDegrees(90));
      none.setCalibError(.1, .1);
      none.setFPS(40);
      none.setExposureTimeMs(10);
      none.setAvgLatencyMs(5);
      none.setLatencyStdDevMs(2);
      propertiesHashMap.put("None", none);
    }
  }
}
