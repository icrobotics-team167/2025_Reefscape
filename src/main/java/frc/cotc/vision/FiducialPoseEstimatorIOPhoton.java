// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.cotc.vision.FiducialPoseEstimatorIO.FiducialPoseEstimatorIOInputs.FiducialPoseEstimate;
import frc.cotc.vision.FiducialPoseEstimatorIO.FiducialPoseEstimatorIOInputs.FiducialPoseEstimate.AprilTag;
import java.util.ArrayList;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

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
                          Pose3d.kZero,
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
  }

  @Override
  public FiducialPoseEstimatorIOConstantsAutoLogged getConstants() {
    return constants;
  }
}
