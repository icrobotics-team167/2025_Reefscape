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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class AlgaeClaw extends SubsystemBase {
  private final AlgaeClawIO io;
  private final AlgaeClawIO.AlgaeClawIOInputs inputs = new AlgaeClawIO.AlgaeClawIOInputs();

  private final LoggedMechanism2d visualization = new LoggedMechanism2d(1, 1);
  private final LoggedMechanismLigament2d wristUpper =
      new LoggedMechanismLigament2d("WristUpper", .25, 0, 2, new Color8Bit(0, 255, 0));
  private final LoggedMechanismLigament2d wristLower =
      new LoggedMechanismLigament2d("WristLower", .25, 0, 2, new Color8Bit(0, 255, 0));

  public AlgaeClaw(AlgaeClawIO io) {
    this.io = io;
    var root = visualization.getRoot("WristPivot", 0.8, .4);
    root.append(wristUpper);
    root.append(wristLower);
  }

  @Override
  public void simulationPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("AlgaeClaw", inputs);

    double wristOpeningAngle = 40;
    double upperAngle = Units.radiansToDegrees(inputs.pivotAngleRad) + (wristOpeningAngle / 2);
    double lowerAngle = Units.radiansToDegrees(inputs.pivotAngleRad) - (wristOpeningAngle / 2);
    wristUpper.setAngle(upperAngle);
    wristLower.setAngle(lowerAngle);

    Logger.recordOutput("AlgaeClaw/Visualization", visualization);
  }

  public Command goToPos(double angleRad) {
    return run(() -> io.setPivotPos(angleRad));
  }

  public Command characterizePivot() {
    var sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(2).per(Second),
                Volts.of(2),
                Seconds.of(2),
                state -> SignalLogger.writeString("SysIDState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> io.characterizePivot(voltage.baseUnitMagnitude()), null, this));
    return sequence(
        runOnce(io::initSysId),
        sysId.quasistatic(SysIdRoutine.Direction.kForward),
        sysId.quasistatic(SysIdRoutine.Direction.kReverse),
        sysId.dynamic(SysIdRoutine.Direction.kForward),
        sysId.dynamic(SysIdRoutine.Direction.kReverse));
  }
}
