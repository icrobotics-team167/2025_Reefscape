// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.arm;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Height;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.cotc.util.MotorCurrentDraws;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import static edu.wpi.first.units.Units.*;

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
    public double posMeters = 0;
    public double velMetersPerSec = 0;

    public MotorCurrentDraws leftMotorCurrentDraws = new MotorCurrentDraws();
    public MotorCurrentDraws rightMotorCurrentDraws = new MotorCurrentDraws();

    @Override
    public void toLog(org.littletonrobotics.junction.LogTable table) {
      table.put("posMeters", posMeters);
      table.put("velMetersPerSec", velMetersPerSec);
      table.put("leftMotorCurrentDraws", MotorCurrentDraws.struct, leftMotorCurrentDraws);
      table.put("rightMotorCurrentDraws", MotorCurrentDraws.struct, rightMotorCurrentDraws);
    }

    @Override
    public void fromLog(org.littletonrobotics.junction.LogTable table) {
      posMeters = table.get("posMeters", 0.0);
      velMetersPerSec = table.get("velMetersPerSec", 0.0);
      leftMotorCurrentDraws = table.get("leftMotorCurrentDraws", MotorCurrentDraws.struct, new MotorCurrentDraws());
      rightMotorCurrentDraws = table.get("rightMotorCurrentDraws", MotorCurrentDraws.struct, new MotorCurrentDraws());
    }
  }

  ElevatorIOConstants getConstants();

  void updateInputs(ElevatorIOInputs inputs);

  void runVoltage(double volts);

  void brake();
}
