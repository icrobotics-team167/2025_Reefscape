// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import frc.cotc.util.MotorCurrentDraws;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ElevatorIO {
  @AutoLog
  class ElevatorIOConstants {
    double kV;
    double kG_firstStage;
    double kG_secondStage;
    double switchPointMeters;
    double maxHeightMeters;
  }

  class ElevatorIOInputs implements LoggableInputs {
    double posMeters;
    double velMetersPerSec;

    MotorCurrentDraws leftMotorCurrentDraws = new MotorCurrentDraws();
    MotorCurrentDraws rightMotorCurrentDraws = new MotorCurrentDraws();

    @Override
    public void toLog(LogTable table) {
      table.put("posMeters", posMeters);
      table.put("velMetersPerSec", velMetersPerSec);
      table.put("leftMotorCurrentDraws", MotorCurrentDraws.struct, leftMotorCurrentDraws);
      table.put("rightMotorCurrentDraws", MotorCurrentDraws.struct, rightMotorCurrentDraws);
    }

    @Override
    public void fromLog(LogTable table) {
      posMeters = table.get("posMeters", 0.0);
      velMetersPerSec = table.get("velMetersPerSec", 0.0);
      leftMotorCurrentDraws =
          table.get("leftMotorCurrentDraws", MotorCurrentDraws.struct, new MotorCurrentDraws());
      rightMotorCurrentDraws =
          table.get("rightMotorCurrentDraws", MotorCurrentDraws.struct, new MotorCurrentDraws());
    }
  }

  default ElevatorIOConstantsAutoLogged getConstants() {
    return new ElevatorIOConstantsAutoLogged();
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void runVoltage(double volts) {}

  default void brake() {}
}
