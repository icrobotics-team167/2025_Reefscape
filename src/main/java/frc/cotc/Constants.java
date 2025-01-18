// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc;

import edu.wpi.first.math.util.Units;

public final class Constants {
  private Constants() {}

  public static final double FIELD_LENGTH_METERS =
      Units.feetToMeters(57) + Units.inchesToMeters(6.875);
  public static final double FIELD_WIDTH_METERS = Units.feetToMeters(26) + Units.inchesToMeters(5);
}
