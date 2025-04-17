// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

class Ramp extends SubsystemBase {
  private final RampIO io;
  private final RampIOInputsAutoLogged inputs = new RampIOInputsAutoLogged();

  public Ramp(RampIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure/Ramp", inputs);
  }

  Command lower() {
    return run(io::lower).until(this::stalled).finallyDo(io::brake).withName("Lower");
  }

  Command raise() {
    return run(io::raise).until(this::stalled).finallyDo(io::brake).withName("Raise");
  }

  Command hold() {
    return run(io::brake).withName("Hold");
  }

  private final Debouncer debouncer = new Debouncer(.1);

  private boolean stalled() {
    var stalled =
        debouncer.calculate(Math.abs(inputs.velRPM) < 75 && inputs.currentDraws.statorCurrent > 5);
    Logger.recordOutput("Superstructure/Ramp/Stalled", stalled);
    return stalled;
  }
}
