// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import frc.cotc.util.MotorCurrentDraws;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface AlgaeClawIO {
  class AlgaeClawIOInputs implements LoggableInputs {
    double pivotAngleRad;
    double pivotVelRadPerSec;

    double wheelPosMeters;
    double wheelVelMetersPerSec;

    MotorCurrentDraws pivotCurrentDraws = new MotorCurrentDraws();
    MotorCurrentDraws wheelCurrentDraws = new MotorCurrentDraws();

    @Override
    public void toLog(LogTable table) {
      table.put("pivotAngleRad", pivotAngleRad);
      table.put("pivotVelRadPerSec", pivotVelRadPerSec);
      table.put("wheelPosMeters", wheelPosMeters);
      table.put("wheelVelMetersPerSec", wheelVelMetersPerSec);
      table.put("pivotCurrentDraws", MotorCurrentDraws.struct, pivotCurrentDraws);
      table.put("wheelCurrentDraws", MotorCurrentDraws.struct, wheelCurrentDraws);
    }

    @Override
    public void fromLog(LogTable table) {
      pivotAngleRad = table.get("pivotAngleRad", 0.0);
      pivotVelRadPerSec = table.get("pivotVelRadPerSec", 0.0);
      wheelPosMeters = table.get("wheelPosMeters", 0.0);
      wheelVelMetersPerSec = table.get("wheelVelMetersPerSec", 0.0);
      pivotCurrentDraws =
          table.get("pivotCurrentDraws", MotorCurrentDraws.struct, new MotorCurrentDraws());
      wheelCurrentDraws =
          table.get("wheelCurrentDraws", MotorCurrentDraws.struct, new MotorCurrentDraws());
    }
  }

  default void updateInputs(AlgaeClawIOInputs inputs) {}

  default void setPivotPos(double angleRad) {}

  default void runIntake(double speedMetersPerSec) {}

  default void characterizePivot(double amps) {}

  default void characterizeIntake(double amps) {}

  default void initSysId() {}
}
