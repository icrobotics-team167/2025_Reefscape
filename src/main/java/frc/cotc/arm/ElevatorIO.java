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
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ElevatorIO {
  @AutoLog
  public class ElevatorIOInputs {

    public Measure<Height> position = Rotations.of(0);

  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setVoltage() {}

  public default void stop() {}
}
