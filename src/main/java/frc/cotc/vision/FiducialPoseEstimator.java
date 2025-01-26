// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.cotc.Constants;
import frc.cotc.vision.FiducialPoseEstimatorIO.FiducialPoseEstimatorIOInputs;
import java.util.ArrayList;
import org.littletonrobotics.junction.Logger;

public class FiducialPoseEstimator {
  static final AprilTagFieldLayout tagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  private final FiducialPoseEstimatorIO io;
  private final FiducialPoseEstimatorIOInputs inputs = new FiducialPoseEstimatorIOInputs();
  private final FiducialPoseEstimatorIOConstantsAutoLogged constants;

  private final String name;

  public record IO(FiducialPoseEstimatorIO io, String name) {}

  public FiducialPoseEstimator(FiducialPoseEstimatorIO io, String name) {
    this.io = io;
    this.name = name;

    constants = io.getConstants();
    Logger.processInputs("Vision/" + name + "/CONSTANTS", constants);
  }

  private final ArrayList<PoseEstimate> estimatesList = new ArrayList<>();
  private final ArrayList<Pose3d> posesUsed = new ArrayList<>();
  private final ArrayList<Pose3d> tagsUsed = new ArrayList<>();

  public PoseEstimate[] poll() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision/" + name, inputs);

    estimatesList.clear();
    posesUsed.clear();
    tagsUsed.clear();
    for (int i = 0; i < inputs.poseEstimates.length; i++) {
      var estimate = inputs.poseEstimates[i];

      switch (estimate.tagsUsed().length) {
        case 0 -> {} // Do nothing
        case 1 -> {
          // TODO: Implement
        }
        default -> {
          // Filter out obviously bad data
          if (Math.abs(estimate.robotPoseEstimate().getZ()) > .05) {
            continue;
          }
          if (estimate.robotPoseEstimate().getX() < 0
              || estimate.robotPoseEstimate().getX() > Constants.FIELD_LENGTH_METERS) {
            continue;
          }
          if (estimate.robotPoseEstimate().getY() < 0
              || estimate.robotPoseEstimate().getY() > Constants.FIELD_WIDTH_METERS) {
            continue;
          }

          double translationalScoresSum = 0;
          double angularScoresSum = 0;
          for (var tag : estimate.tagsUsed()) {
            tagsUsed.add(tag.location());
            var tagDistance = tag.distanceToCamera();

            translationalScoresSum += .1 * tagDistance * tagDistance;
            angularScoresSum += .05 * tagDistance * tagDistance;
          }

          var translationalDivisor = Math.pow(estimate.tagsUsed().length, 2);
          var angularDivisor = Math.pow(estimate.tagsUsed().length, 1.5);

          estimatesList.add(
              new PoseEstimate(
                  estimate.robotPoseEstimate().toPose2d(),
                  estimate.timestamp(),
                  translationalScoresSum / translationalDivisor,
                  angularScoresSum / angularDivisor));
        }
      }
    }

    Logger.recordOutput("Vision/" + name + "/Filtered Poses", posesUsed.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/" + name + "/Tags Used", tagsUsed.toArray(new Pose3d[0]));
    return estimatesList.toArray(new PoseEstimate[0]);
  }

  public record PoseEstimate(
      Pose2d pose, double timestamp, double translationalStdDevs, double yawStdDevs) {}
}
