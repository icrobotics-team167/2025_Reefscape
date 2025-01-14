// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class CoralElevator extends SubsystemBase {
  private final CoralElevatorIO io;
  private final CoralElevatorIO.CoralElevatorIOInputs inputs =
      new CoralElevatorIO.CoralElevatorIOInputs();

  public CoralElevator(CoralElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("CoralElevator", inputs);
  }

  public Command goToPos(double posMeters) {
    return run(() -> io.goToPos(posMeters));
  }

  public Command characterize() {
    var sysid =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Second),
                Volts.of(12),
                Seconds.of(6),
                state -> SignalLogger.writeString("SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.characterize(voltage.baseUnitMagnitude()), null, this));

    return sequence(
        sysid.quasistatic(SysIdRoutine.Direction.kForward),
        sysid.quasistatic(SysIdRoutine.Direction.kReverse),
        sysid.dynamic(SysIdRoutine.Direction.kForward).withTimeout(.5),
        sysid.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(.5));
  }
}
