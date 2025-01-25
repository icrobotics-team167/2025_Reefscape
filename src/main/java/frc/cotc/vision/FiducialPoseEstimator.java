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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

  @FunctionalInterface
  public interface GyroYawGetter {
    Rotation2d getYawAtTime(double time);
  }

  private final GyroYawGetter gyroYawGetter;

  public FiducialPoseEstimator(
      FiducialPoseEstimatorIO io, String name, GyroYawGetter gyroYawGetter) {
    this.io = io;
    this.name = name;

    constants = io.getConstants();
    Logger.processInputs("Vision/" + name + "/CONSTANTS", constants);

    this.gyroYawGetter = gyroYawGetter;
  }

  private final ArrayList<PoseEstimate> estimatesList = new ArrayList<>();

  public PoseEstimate[] poll() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision/" + name, inputs);

    estimatesList.clear();
    for (int i = 0; i < inputs.poseEstimates.length; i++) {
      var estimate = inputs.poseEstimates[i];

      switch (estimate.tagsUsed().length) {
        case 0 -> {} // Do nothing
        case 1 -> {
          var robotGyroYaw = gyroYawGetter.getYawAtTime(estimate.timestamp());
          if (robotGyroYaw == null) {
            continue;
          }

          var tag = estimate.tagsUsed()[0];
          // Filter out ambiguous data
          if (tag.ambiguity() > .3) {
            continue;
          }

          var optional = tagLayout.getTagPose(tag.id());
          if (optional.isEmpty()) {
            continue;
          }

          var tagPosition = optional.get().getTranslation().toTranslation2d();

          // Tx pitch is relative to camera so add camera pitch to make it world pitch
          var worldPitchRad = constants.robotToCameraTransform.getRotation().getY() + tag.tx();

          // Filter out the data if it results in a z height that's too high/too low
          if (Math.abs(tag.distanceToCamera() * Math.sin(worldPitchRad)) > .05) {
            continue;
          }

          // Ty yaw is relative to camera so add camera yaw and robot yaw to make it world yaw
          var worldYaw =
              robotGyroYaw.plus(
                  new Rotation2d(constants.robotToCameraTransform.getRotation().getZ() + tag.ty()));

          // Solve for translational distance (we know hypotenuse length and angle)
          var translationalDistance = tag.distanceToCamera() * Math.cos(worldPitchRad);

          var robotPose =
              new Pose2d(
                  tagPosition.plus(new Translation2d(-translationalDistance, worldYaw)),
                  robotGyroYaw);

          // Scales with the cube of distance (translational sensor noise scales with the square
          // and angular sensor noise scales proportionally, and we use both)
          var translationalStdDev =
              .1 * tag.distanceToCamera() * tag.distanceToCamera() * tag.distanceToCamera();
          // Yaw std dev is constant as we use the gyro measurement.
          estimatesList.add(
              new PoseEstimate(robotPose, estimate.timestamp(), translationalStdDev, 2));
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

          var tagTranslationalStdDevs = new double[estimate.tagsUsed().length];
          var tagAngularStdDevs = new double[estimate.tagsUsed().length];
          for (int j = 0; j < tagTranslationalStdDevs.length; j++) {
            var tag = estimate.tagsUsed()[i];
            
          }

          estimatesList.add(
              new PoseEstimate(
                  estimate.robotPoseEstimate().toPose2d(), estimate.timestamp(), .1, .1));
        }
      }
    }
    return estimatesList.toArray(new PoseEstimate[0]);
  }

  public record PoseEstimate(
      Pose2d pose, double timestamp, double translationalStdDevs, double yawStdDevs) {}
}
