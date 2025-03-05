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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

public class Superstructure extends SubsystemBase {
  private final Elevator elevator;
  private final CoralOuttake coralOuttake;
  private final Ramp ramp;
  private final AlgaePivot algaePivot;
  private final AlgaeIntake algaeIntake;

  public Superstructure(ElevatorIO elevatorIO, CoralOuttakeIO coralOuttakeIO, RampIO rampIO,
                        AlgaePivotIO algaePivotIO, AlgaeIntakeIO algaeIntakeIO) {
    elevator = new Elevator(elevatorIO);
    coralOuttake = new CoralOuttake(coralOuttakeIO);
    ramp = new Ramp(rampIO);
    algaePivot = new AlgaePivot(algaePivotIO);
    algaeIntake = new AlgaeIntake(algaeIntakeIO);

    elevator.setDefaultCommand(elevator.retract());
    coralOuttake.setDefaultCommand(coralOuttake.intake());
    ramp.setDefaultCommand(ramp.hold());
    RobotModeTriggers.disabled().onFalse(ramp.lower());
    algaePivot.setDefaultCommand(algaePivot.stow());
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

  public Command readyClimb() {
    return expose(ramp.raise());
  }

  public boolean hasCoral() {
    return coralOuttake.hasCoral();
  }

  public Trigger coralStuck() {
    return new Trigger(coralOuttake::coralStuck);
  }

  private Command expose(Command internal) {
    var proxied = internal.asProxy();
    proxied.addRequirements(this);
    return proxied;
  }
}
