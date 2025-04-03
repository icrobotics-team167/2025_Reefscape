// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import frc.cotc.util.MotorCurrentDraws;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOConstants {
    double switchPointMeters;
    double maxHeightMeters;
  }

  @AutoLog
  class ElevatorIOInputs {
    double posMeters;
    double velMetersPerSec;

    MotorCurrentDraws leftMotorCurrentDraws = new MotorCurrentDraws();
    MotorCurrentDraws rightMotorCurrentDraws = new MotorCurrentDraws();
  }

  default ElevatorIOConstantsAutoLogged getConstants() {
    return new ElevatorIOConstantsAutoLogged();
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setTargetPos(double posMeters) {}

  default void brake() {}
}
