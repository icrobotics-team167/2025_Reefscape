// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

class AlgaePivot extends SubsystemBase {
  private final AlgaePivotIO io;
  private final AlgaePivotIO.AlgaeClawIOInputs inputs = new AlgaePivotIO.AlgaeClawIOInputs();

  AlgaePivot(AlgaePivotIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure/AlgaeClaw", inputs);
  }

  Command intake() {
    return run(() -> setTargetPos(Units.degreesToRadians(-10))).withName("Intake");
  }

  Command processor() {
    return run(() -> setTargetPos(Units.degreesToRadians(-50))).withName("Processor");
  }

  Command barge() {
    return run(() -> setTargetPos(Units.degreesToRadians(110))).withName("Barge");
  }

  Command stow() {
    return run(() -> setTargetPos(Units.degreesToRadians(-75))).withName("Stow");
  }

  Command rezero() {
    return run(io::lowerSlow).finallyDo(io::resetAlgae);
  }

  private double targetPosRad;

  private void setTargetPos(double posRad) {
    targetPosRad = posRad;
    io.setTargetPos(posRad);
    Logger.recordOutput("Superstructure/AlgaeClaw/Pivot/targetPosRad", targetPosRad);
  }

  boolean atTargetPos() {
    return MathUtil.isNear(targetPosRad, inputs.posRad, Units.degreesToRadians(5))
        && MathUtil.isNear(0, inputs.velRadPerSec, Units.degreesToRadians(10));
  }
}
