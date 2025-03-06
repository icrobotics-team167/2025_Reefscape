// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.cotc.util.Mechanism;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Superstructure extends Mechanism {
  private final Elevator elevator;
  private final CoralOuttake coralOuttake;
  private final Ramp ramp;
  private final AlgaeClaw algaeClaw;

  public Superstructure(
      ElevatorIO elevatorIO,
      CoralOuttakeIO coralOuttakeIO,
      RampIO rampIO,
      AlgaePivotIO algaePivotIO,
      AlgaeIntakeIO algaeIntakeIO) {
    elevator = new Elevator(elevatorIO);
    coralOuttake = new CoralOuttake(coralOuttakeIO);
    ramp = new Ramp(rampIO);
    algaeClaw = new AlgaeClaw(algaePivotIO, algaeIntakeIO);

    elevator.setDefaultCommand(elevator.retract());
    coralOuttake.setDefaultCommand(coralOuttake.intake());
    ramp.setDefaultCommand(ramp.hold());
    RobotModeTriggers.disabled().onFalse(ramp.lower());
  }

  public Command lvl1() {
    return expose(
            deadline(
                waitUntil(elevator::atTargetPos).andThen(coralOuttake.score(), waitSeconds(.5)),
                elevator.lvl1()))
        .withName("Lvl 1 Scoring");
  }

  public Command lvl2(BooleanSupplier driveBaseAtTarget) {
    return expose(
            deadline(
                waitUntil(() -> driveBaseAtTarget.getAsBoolean() && elevator.atTargetPos())
                    .andThen(coralOuttake.score(), waitSeconds(.5)),
                elevator.lvl2()))
        .withName("Lvl 2 Scoring");
  }

  public Command lvl3(BooleanSupplier driveBaseAtTarget) {
    return expose(
            deadline(
                waitUntil(() -> driveBaseAtTarget.getAsBoolean() && elevator.atTargetPos())
                    .andThen(coralOuttake.score(), waitSeconds(.5)),
                elevator.lvl3()))
        .withName("Lvl 3 Scoring");
  }

  public Command lvl4(BooleanSupplier driveBaseAtTarget) {
    return expose(
            deadline(
                waitUntil(() -> driveBaseAtTarget.getAsBoolean() && elevator.atTargetPos())
                    .andThen(coralOuttake.score(), waitSeconds(.75)),
                elevator.lvl4()))
        .withName("Lvl 4 Scoring");
  }

  public Command intakeCoral() {
    return expose(coralOuttake.intake()).withName("Intake");
  }

  public Command ejectStuckCoral() {
    return expose(
            coralOuttake
                .agitate()
                .withDeadline(waitUntil(() -> !coralOuttake.coralStuck()).andThen(waitSeconds(.5)))
                .withName("Eject Stuck Coral"))
        .withName("Eject Stuck Coral");
  }

  public Command intakeLowAlgae() {
    return expose(algaeClaw.reefIntake()).withName("Intake Low Algae");
  }

  public Command intakeHighAlgae() {
    return expose(
            parallel(elevator.highAlgae(), algaeClaw.reefIntake()).withName("Intake High Algae"))
        .withName("Intake High Algae");
  }

  public Command lowAlgaeManualOverride(DoubleSupplier control) {
    return expose(algaeClaw.)
  }

  public Command processorScore() {
    return expose(algaeClaw.processorScore()).withName("Processor Score");
  }

  public Command netScore() {
    return expose(algaeClaw.bargeScore(elevator::atTargetPos).deadlineFor(elevator.lvl4()));
  }

  public Command raiseIfHasAlgae() {
    return expose(algaeClaw.holdIfHasAlgae());
  }

  public Command readyClimb() {
    return expose(ramp.raise());
  }

  public Trigger coralStuck() {
    return new Trigger(coralOuttake::coralStuck);
  }
}
