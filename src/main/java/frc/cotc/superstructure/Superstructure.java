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

public class Superstructure extends Mechanism {
  private final Elevator elevator;
  private final CoralOuttake coralOuttake;
  private final Ramp ramp;
  private final Climber climber;

  public Superstructure(ElevatorIO elevatorIO, CoralOuttakeIO coralOuttakeIO, RampIO rampIO) {
    elevator = new Elevator(elevatorIO);
    coralOuttake = new CoralOuttake(coralOuttakeIO);
    ramp = new Ramp(rampIO);

    elevator.setDefaultCommand(elevator.retract());
    coralOuttake.setDefaultCommand(coralOuttake.intake());
    ramp.setDefaultCommand(ramp.hold());
    RobotModeTriggers.disabled().onFalse(ramp.lower().alongWith(climber.deployStart()));
  }

  public Command lvl2(BooleanSupplier driveBaseAtTarget) {
    return expose(
            elevator
                .lvl2()
                .withDeadline(
                    waitUntil(() -> driveBaseAtTarget.getAsBoolean() && elevator.atTargetPos())
                        .andThen(coralOuttake.scoreFast().asProxy()))
                .withName("Lvl 2 Scoring"))
        .withName("Lvl 2 Scoring");
  }

  public Command lvl3(BooleanSupplier driveBaseAtTarget) {
    return expose(
            elevator
                .lvl3()
                .withDeadline(
                    waitUntil(() -> driveBaseAtTarget.getAsBoolean() && elevator.atTargetPos())
                        .andThen(coralOuttake.scoreFast().asProxy()))
                .withName("Lvl 3 Scoring"))
        .withName("Lvl 3 Scoring");
  }

  public Command lvl4(BooleanSupplier driveBaseAtTarget) {
    return expose(
            elevator
                .lvl4()
                .withDeadline(
                    waitUntil(() -> driveBaseAtTarget.getAsBoolean() && elevator.atTargetPos())
                        .andThen(coralOuttake.scoreSlow().asProxy()))
                .withName("Lvl 4 Scoring"))
        .withName("Lvl 4 Scoring");
  }

  public Command ejectStuckCoral() {
    return expose(
            coralOuttake
                .agitate()
                .withDeadline(waitUntil(() -> !coralOuttake.coralStuck()).andThen(waitSeconds(.5)))
                .withName("Eject Stuck Coral"))
        .withName("Eject Stuck Coral");
  }

  public Command highAlgae() {
    return expose(elevator.highAlgae());
  }

  public Command net() {
    return expose(elevator.net());
  }

  private boolean climberDeployed = false;

  public boolean isClimberDeployed() {
    return climberDeployed;
  }

  public Command readyClimb() {
    return expose(
        runOnce(() -> climberDeployed = true).andThen(parallel(ramp.raise(), climber.deploy())));
  }

  public Command climb() {
    return expose(climber.climb());
  }

  public Command raiseClimber() {
    return expose(climber.deploy());
  }

  public Trigger coralStuck() {
    return new Trigger(coralOuttake::coralStuck);
  }

  public boolean hasCoral() {
    return coralOuttake.hasCoral();
  }
}
