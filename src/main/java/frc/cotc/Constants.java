// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc;

import edu.wpi.first.math.geometry.Translation2d;
import frc.cotc.vision.FiducialPoseEstimator;

public final class Constants {
  private Constants() {}

  public static final double FIELD_LENGTH_METERS = FiducialPoseEstimator.tagLayout.getFieldLength();
  public static final double FIELD_WIDTH_METERS = FiducialPoseEstimator.tagLayout.getFieldWidth();
  public static final Translation2d FIELD_CENTER =
      new Translation2d(FIELD_LENGTH_METERS / 2, FIELD_WIDTH_METERS / 2);
}
