// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.cotc.Constants;
import frc.cotc.vision.FiducialPoseEstimatorIO.FiducialPoseEstimatorIOInputs;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class FiducialPoseEstimator {
  static final AprilTagFieldLayout tagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  private final FiducialPoseEstimatorIO io;
  private final FiducialPoseEstimatorIOInputs inputs = new FiducialPoseEstimatorIOInputs();
  private final Transform3d robotToCameraTransform;
  private final Transform2d cameraToRobotTransform2d;

  private final GyroYawGetter gyroYawGetter;
  private final Supplier<Pose2d> currentPoseEstimateSupplier;

  private final String name;

  public record IO(FiducialPoseEstimatorIO io, String name) {}

  @FunctionalInterface
  public interface GyroYawGetter {
    Rotation2d get(double timestamp);
  }

  public FiducialPoseEstimator(
      FiducialPoseEstimatorIO io,
      String name,
      GyroYawGetter gyroYawGetter,
      Supplier<Pose2d> currentPoseEstimateSupplier) {
    this.io = io;
    this.name = name;

    var constants = io.getConstants();
    Logger.processInputs("Vision/" + name + "/CONSTANTS", constants);
    robotToCameraTransform = constants.robotToCameraTransform;
    var cameraToRobotTransform3d = constants.robotToCameraTransform.inverse();
    cameraToRobotTransform2d =
        new Transform2d(
            cameraToRobotTransform3d.getTranslation().toTranslation2d(),
            cameraToRobotTransform3d.getRotation().toRotation2d());
    this.gyroYawGetter = gyroYawGetter;
    this.currentPoseEstimateSupplier = currentPoseEstimateSupplier;
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
          var tag = estimate.tagsUsed()[0];

          // Discard if key data is missing (position of tag and gyro yaw)
          var tagPoseOptional = tagLayout.getTagPose(tag.id());
          if (tagPoseOptional.isEmpty()) {
            continue;
          }
          var gyroYaw = gyroYawGetter.get(estimate.timestamp());
          if (gyroYaw == null) {
            continue;
          }
          var tagPose = tagPoseOptional.get();

          // Algorithm adapted from 6328 Mechanical Advantage
          // https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2025-build-thread/477314/85
          var cameraToTagTranslation =
              new Pose3d(
                      Translation3d.kZero,
                      new Rotation3d(
                          0, Units.degreesToRadians(-tag.ty()), Units.degreesToRadians(-tag.tx())))
                  .transformBy(
                      new Transform3d(
                          new Translation3d(tag.distanceToCamera(), 0, 0), Rotation3d.kZero))
                  .getTranslation()
                  .rotateBy(new Rotation3d(0, robotToCameraTransform.getRotation().getY(), 0))
                  .toTranslation2d();
          var cameraToTagRotation =
              gyroYaw.plus(
                  robotToCameraTransform
                      .getRotation()
                      .toRotation2d()
                      .plus(cameraToTagTranslation.getAngle()));

          var cameraTranslation =
              new Pose2d(
                      tagPose.getTranslation().toTranslation2d(),
                      cameraToTagRotation.plus(Rotation2d.kPi))
                  .transformBy(
                      new Transform2d(cameraToTagTranslation.getNorm(), 0, Rotation2d.kZero))
                  .getTranslation();
          var robotPose =
              new Pose2d(
                      cameraTranslation,
                      gyroYaw.plus(robotToCameraTransform.getRotation().toRotation2d()))
                  .transformBy(cameraToRobotTransform2d);
          robotPose = new Pose2d(robotPose.getTranslation(), gyroYaw);

          // Filter out obviously bad data
          if (robotPose.getX() < 0 || robotPose.getX() > Constants.FIELD_LENGTH_METERS) {
            continue;
          }
          if (robotPose.getY() < 0 || robotPose.getY() > Constants.FIELD_WIDTH_METERS) {
            continue;
          }

          // If it's too far off the pose estimate that's already in, discard
          // Prevents bad data from bad initial conditions from affecting estimates
          var delta = robotPose.minus(currentPoseEstimateSupplier.get());
          if (delta.getTranslation().getNorm() > .025) {
            continue;
          }
          if (delta.getRotation().getDegrees() > 2) {
            continue;
          }

          tagsUsed.add(tagPose);
          posesUsed.add(new Pose3d(robotPose));

          // Heavy distrust compared multi-tag SolvePnp, due to the inherent lack of information
          // usable in the solve
          var stdDev =
              .5 * tag.distanceToCamera() * tag.distanceToCamera() * tag.distanceToCamera();

          estimatesList.add(new PoseEstimate(robotPose, estimate.timestamp(), stdDev, .001));
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

            translationalScoresSum += .25 * tagDistance * tagDistance;
            angularScoresSum += .0001 * tagDistance * tagDistance;
          }

          var translationalDivisor = Math.pow(estimate.tagsUsed().length, 1.5);
          var angularDivisor = Math.pow(estimate.tagsUsed().length, 3);

          posesUsed.add(estimate.robotPoseEstimate());

          estimatesList.add(
              new PoseEstimate(
                  estimate.robotPoseEstimate().toPose2d(),
                  estimate.timestamp(),
                  translationalScoresSum / translationalDivisor,
                  angularScoresSum / angularDivisor));
        }
      }
    }

    Logger.recordOutput("Vision/" + name + "/Poses Used", posesUsed.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/" + name + "/Tags Used", tagsUsed.toArray(new Pose3d[0]));
    return estimatesList.toArray(new PoseEstimate[0]);
  }

  public record PoseEstimate(
      Pose2d pose, double timestamp, double translationalStdDevs, double yawStdDevs) {}
}
