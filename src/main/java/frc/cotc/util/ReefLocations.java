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

public final class ReefLocations {
  private ReefLocations() {}

  public static final Pose2d[] BLUE_POSES;
  public static final Pose2d[] RED_POSES;

  static {
    var BLUE_REEF_LOCATION = new Translation2d(4.495, Constants.FIELD_WIDTH_METERS / 2);
    var FIELD_CENTER =
        new Translation2d(Constants.FIELD_LENGTH_METERS / 2, Constants.FIELD_WIDTH_METERS / 2);

    var A =
        new Pose2d(
            BLUE_REEF_LOCATION.getX() - 1.265,
            Constants.FIELD_WIDTH_METERS / 2 + .165,
            Rotation2d.kZero);
    var B =
        new Pose2d(
            BLUE_REEF_LOCATION.getX() - 1.265,
            Constants.FIELD_WIDTH_METERS / 2 - .165,
            Rotation2d.kZero);

    BLUE_POSES = new Pose2d[12];
    BLUE_POSES[0] = A;
    BLUE_POSES[1] = B;
    for (int i = 2; i < 12; i += 2) {
      var rotAngle = Rotation2d.fromDegrees(30 * i);
      BLUE_POSES[i] =
          new Pose2d(
              A.getTranslation().rotateAround(BLUE_REEF_LOCATION, rotAngle),
              A.getRotation().rotateBy(rotAngle));
      BLUE_POSES[i + 1] =
          new Pose2d(
              B.getTranslation().rotateAround(BLUE_REEF_LOCATION, rotAngle),
              B.getRotation().rotateBy(rotAngle));
    }

    RED_POSES = new Pose2d[12];
    for (int i = 0; i < 12; i++) {
      RED_POSES[i] =
          new Pose2d(
              BLUE_POSES[i].getTranslation().rotateAround(FIELD_CENTER, Rotation2d.kPi),
              BLUE_POSES[i].getRotation().rotateBy(Rotation2d.kPi));
    }
  }
}
