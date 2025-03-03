// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
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
                          tag.yaw,
                          tag.pitch,
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
      cameraSim.setMaxSightRange(6);
      visionSystemSim.addCamera(cameraSim, transform);
    }

    public static void update() {
      if (Robot.groundTruthPoseSupplier != null && visionSystemSim != null) {
        visionSystemSim.update(Robot.groundTruthPoseSupplier.get());
      }
    }

    private static final HashMap<String, SimCameraProperties> propertiesHashMap = new HashMap<>();

    static {
      var none = new SimCameraProperties();
      none.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
      none.setCalibError(.3, .1);
      none.setFPS(20);
      none.setExposureTimeMs(10);
      none.setAvgLatencyMs(5);
      none.setLatencyStdDevMs(2);
      propertiesHashMap.put("None", none);

      var frontLeft = new SimCameraProperties();
      frontLeft.setCalibration(
          1280,
          800,
          MatBuilder.fill(
              Nat.N3(),
              Nat.N3(),
              917.7435751,
              0.0,
              916.4687196,
              0.0,
              645.1229926,
              438.7684688,
              0.0,
              0.0,
              1.0),
          MatBuilder.fill(
              Nat.N8(),
              Nat.N1(),
              0.04160394239,
              -0.05934898907,
              -0.0002104558183,
              -0.001798832851,
              0.005054943256,
              -0.001768082509,
              0.005583424767,
              0.0009199079055));
      frontLeft.setCalibError(.2, .01);
      frontLeft.setFPS(20);
      frontLeft.setExposureTimeMs(10);
      frontLeft.setAvgLatencyMs(5);
      frontLeft.setLatencyStdDevMs(2);
      propertiesHashMap.put("FrontLeftCamera", frontLeft);

      var frontRight = new SimCameraProperties();
      frontLeft.setCalibration(
          1280,
          800,
          MatBuilder.fill(
              Nat.N3(),
              Nat.N3(),
              913.4991235,
              0.0,
              912.2663355,
              0.0,
              656.9013349,
              380.7381692,
              0.0,
              0.0,
              1.0),
          MatBuilder.fill(
              Nat.N8(),
              Nat.N1(),
              0.04363956086,
              -0.06962515596,
              -0.0004791594092,
              -0.001254755397,
              0.007056855264,
              -0.001138138589,
              -0.0003849712931,
              -0.001715779282));
      frontLeft.setCalibError(.3, .01);
      frontLeft.setFPS(20);
      frontLeft.setExposureTimeMs(10);
      frontLeft.setAvgLatencyMs(5);
      frontLeft.setLatencyStdDevMs(2);
      propertiesHashMap.put("FrontRightCamera", frontRight);
    }
  }
}
