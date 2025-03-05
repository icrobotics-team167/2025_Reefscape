// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Superstructure extends SubsystemBase {
  private final Elevator elevator;
  private final CoralOuttake coralOuttake;
  private final Ramp ramp;

  public Superstructure(ElevatorIO elevatorIO, CoralOuttakeIO coralOuttakeIO, RampIO rampIO) {
    elevator = new Elevator(elevatorIO);
    coralOuttake = new CoralOuttake(coralOuttakeIO);
    ramp = new Ramp(rampIO);

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

  public Command elevatorManualControl(DoubleSupplier control) {
    return expose(elevator.manualControl(control));
  }

  public Command intake() {
    return expose(coralOuttake.intake()).withName("Intake");
  }

  public Command agitate() {
    return expose(coralOuttake.agitate()).withName("Agitate");
  }

  public Command readyClimb() {
    return expose(ramp.raise());
  }

  public boolean hasCoral() {
    return coralOuttake.hasCoral();
  }

  private Command expose(Command internal) {
    var proxied = internal.asProxy();
    proxied.addRequirements(this);
    return proxied;
  }
}
