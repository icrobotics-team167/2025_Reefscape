// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.arm;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Superstructure extends SubsystemBase {
  private final Elevator elevator;
  private final CoralOuttake coralOuttake;

  public Superstructure(ElevatorIO elevatorIO, CoralOuttakeIO coralOuttakeIO) {
    this.elevator = new Elevator(elevatorIO);
    this.coralOuttake = new CoralOuttake(coralOuttakeIO);

    elevator.setDefaultCommand(elevator.retract());
  }

  public Command lvl1() {
    return expose(
        deadline(
            waitUntil(elevator::atTargetPos).andThen(coralOuttake.out(), waitSeconds(.25)),
            elevator.lvl1()));
  }

  public Command lvl2(BooleanSupplier driveBaseAtTarget) {
    return expose(
        deadline(
            waitUntil(() -> driveBaseAtTarget.getAsBoolean() && elevator.atTargetPos())
                .andThen(coralOuttake.out()),
            elevator.lvl2()));
  }

  public Command lvl3(BooleanSupplier driveBaseAtTarget) {
    return expose(
        deadline(
            waitUntil(() -> driveBaseAtTarget.getAsBoolean() && elevator.atTargetPos())
                .andThen(coralOuttake.out()),
            elevator.lvl3()));
  }

  public Command lvl4(BooleanSupplier driveBaseAtTarget) {
    return expose(
        deadline(
            waitUntil(() -> driveBaseAtTarget.getAsBoolean() && elevator.atTargetPos())
                .andThen(coralOuttake.out(), waitSeconds(.25)),
            elevator.lvl4()));
  }

  public Command elevatorManualControl(DoubleSupplier control) {
    return expose(elevator.manualControl(control));
  }

  public Command intake() {
    return expose(coralOuttake.in());
  }

  private Command expose(Command internal) {
    var proxied = internal.asProxy();
    proxied.addRequirements(this);
    return proxied;
  }
}
