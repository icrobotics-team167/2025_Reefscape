// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.cotc.util.Mechanism;

class AlgaeClaw extends Mechanism {
  private final AlgaePivot pivot;
  private final AlgaeRollers rollers;

  AlgaeClaw(AlgaePivotIO pivotIO, AlgaeRollersIO rollersIO) {
    pivot = new AlgaePivot(pivotIO);
    rollers = new AlgaeRollers(rollersIO);

    if (rollersIO instanceof AlgaeRollersIOSim simRollers) {
      simRollers.atTargetAngle = pivot::atIntakeAngle;
    }
  }

  Command intake() {
    return expose(parallel(pivot.intake(), rollers.intake()).withName("Intake")).withName("Intake");
  }

  Command stow() {
    return expose(pivot.stow()).withName("Stow");
  }

  Command hold() {
    return expose(parallel(pivot.hold(), rollers.intake()).withName("Hold")).withName("Hold");
  }

  Command eject() {
    return expose(parallel(pivot.hold(), rollers.eject()).withName("Eject")).withName("Eject");
  }

  boolean hasAlgae() {
    return rollers.hasAlgae();
  }
}
