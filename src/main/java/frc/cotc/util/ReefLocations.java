// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.cotc.Constants;
import frc.cotc.Robot;
import frc.cotc.vision.FiducialPoseEstimator;
import org.littletonrobotics.junction.Logger;

public final class ReefLocations {
  private ReefLocations() {}

  public static final Pose2d[] BLUE_ALGAE_POSES;
  public static final Pose2d[] BLUE_BRANCH_POSES;
  public static final Pose2d[] RED_ALGAE_POSES;
  public static final Pose2d[] RED_BRANCH_POSES;

  public static final Translation2d BLUE_REEF;
  public static final Translation2d RED_REEF;

  static {
    //noinspection OptionalGetWithoutIsPresent
    double tag18X = FiducialPoseEstimator.tagLayout.getTagPose(18).get().getX();
    //noinspection OptionalGetWithoutIsPresent
    double tag21X = FiducialPoseEstimator.tagLayout.getTagPose(21).get().getX();
    BLUE_REEF = new Translation2d((tag18X + tag21X) / 2, Constants.FIELD_WIDTH_METERS / 2);

    var AB_faceTranslation =
        new Translation2d(
            tag18X - (Constants.BUMPER_THICKNESS_METERS + (Constants.FRAME_LENGTH_METERS / 2)),
            Constants.FIELD_WIDTH_METERS / 2);
    var A = new Pose2d(AB_faceTranslation.plus(new Translation2d(0, .165)), Rotation2d.kZero);
    var B = new Pose2d(AB_faceTranslation.plus(new Translation2d(0, -.165)), Rotation2d.kZero);

    BLUE_BRANCH_POSES = new Pose2d[12];
    BLUE_BRANCH_POSES[0] = A;
    BLUE_BRANCH_POSES[1] = B;
    for (int i = 2; i < 12; i += 2) {
      var rotAngle = Rotation2d.fromDegrees(30 * i);
      BLUE_BRANCH_POSES[i] = A.rotateAround(BLUE_REEF, rotAngle);
      BLUE_BRANCH_POSES[i + 1] = B.rotateAround(BLUE_REEF, rotAngle);
    }

    RED_REEF = BLUE_REEF.rotateAround(Constants.FIELD_CENTER, Rotation2d.kPi);
    RED_BRANCH_POSES = new Pose2d[12];
    for (int i = 0; i < 12; i++) {
      RED_BRANCH_POSES[i] =
          BLUE_BRANCH_POSES[i].rotateAround(Constants.FIELD_CENTER, Rotation2d.kPi);
    }

    BLUE_ALGAE_POSES = new Pose2d[6];
    RED_ALGAE_POSES = new Pose2d[6];
    BLUE_ALGAE_POSES[0] = new Pose2d(AB_faceTranslation, Rotation2d.kZero);
    RED_ALGAE_POSES[0] = BLUE_ALGAE_POSES[0].rotateAround(Constants.FIELD_CENTER, Rotation2d.kPi);
    for (int i = 1; i < 6; i++) {
      BLUE_ALGAE_POSES[i] =
          BLUE_ALGAE_POSES[0].rotateAround(BLUE_REEF, Rotation2d.fromDegrees(i * 60));
      RED_ALGAE_POSES[i] = BLUE_ALGAE_POSES[i].rotateAround(Constants.FIELD_CENTER, Rotation2d.kPi);
    }
  }

  public enum ReefBranch {
    A(0),
    B(1),
    C(2),
    D(3),
    E(4),
    F(5),
    G(6),
    H(7),
    I(8),
    J(9),
    K(10),
    L(11);

    final int id;

    ReefBranch(int id) {
      this.id = id;
    }
  }

  public static Pose2d getScoringLocation(ReefBranch reefBranch) {
    return (Robot.isOnRed() ? RED_BRANCH_POSES : BLUE_BRANCH_POSES)[reefBranch.id];
  }

  public static void log() {
    Logger.recordOutput("Reef Scoring Locations/Blue", BLUE_BRANCH_POSES);
    Logger.recordOutput("Reef Scoring Locations/Red", RED_BRANCH_POSES);
    Logger.recordOutput("Reef Scoring Locations/Blue Reef Center", BLUE_REEF);
    Logger.recordOutput("Reef Scoring Locations/Red Reef Center", RED_REEF);
  }
}
