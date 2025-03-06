// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.cotc.util.Mechanism;
import java.util.function.BooleanSupplier;

public class AlgaeClaw extends Mechanism {
  private final AlgaePivot algaePivot;
  private final AlgaeIntake algaeIntake;

  AlgaeClaw(AlgaePivotIO pivotIO, AlgaeIntakeIO intakeIO) {
    algaePivot = new AlgaePivot(pivotIO);
    algaeIntake = new AlgaeIntake(intakeIO);

    algaePivot.setDefaultCommand(algaePivot.stow());
  }

  Command reefIntake() {
    return expose(parallel(algaePivot.intake(), algaeIntake.intake()).withName("Reef Intake"))
        .withName("Reef Intake");
  }

  Command bargeScore(BooleanSupplier atTargetHeight) {
    return expose(
            waitUntil(() -> atTargetHeight.getAsBoolean() && algaePivot.atTargetPos())
                .andThen(algaeIntake.outtake())
                .deadlineFor(algaePivot.barge())
                .withName("Barge Score"))
        .withName("Barge Score");
  }

  Command processorScore() {
    return expose(
            waitUntil(algaePivot::atTargetPos)
                .andThen(algaeIntake.outtake())
                .deadlineFor(algaePivot.processor())
                .withName("Processor Score"))
        .withName("Processor Score");
  }

  Command holdIfHasAlgae() {
    return either(expose(new ScheduleCommand(algaePivot.barge())), none(), algaeIntake::hasAlgae)
        .withName("Hold if has algae");
  }

  Trigger hasAlgae() {
    return new Trigger(algaeIntake::hasAlgae);
  }
}
