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
  private final AlgaeClaw algaeClaw;
  private final Ramp ramp;
  private final Climber climber;

  public Superstructure(
      ElevatorIO elevatorIO,
      CoralOuttakeIO coralOuttakeIO,
      AlgaePivotIO algaePivotIO,
      AlgaeRollersIO algaeRollersIO,
      RampIO rampIO,
      ClimberIO climberIO) {
    elevator = new Elevator(elevatorIO);
    coralOuttake = new CoralOuttake(coralOuttakeIO);
    algaeClaw = new AlgaeClaw(algaePivotIO, algaeRollersIO);
    ramp = new Ramp(rampIO);
    climber = new Climber(climberIO);

    elevator.setDefaultCommand(elevator.retract());
    coralOuttake.setDefaultCommand(coralOuttake.intake());
    ramp.setDefaultCommand(ramp.hold());
    RobotModeTriggers.disabled().onFalse(ramp.lower());
    algaeClaw.setDefaultCommand(either(algaeClaw.hold(), algaeClaw.stow(), algaeClaw::hasAlgae));

    if (algaeRollersIO instanceof AlgaeRollersIOSim simRollers) {
      simRollers.atTargetHeight = elevator::atTargetPos;
    }
  }

  public Command lvl2(BooleanSupplier driveBaseAtTarget) {
    return expose(
            elevator
                .lvl2()
                .withDeadline(
                    waitUntil(() -> driveBaseAtTarget.getAsBoolean() && elevator.atTargetPos())
                        .andThen(coralOuttake.scoreFast().asProxy())
                        .withName("Score On Branch"))
                .withName("Lvl 2 Scoring"))
        .withName("Lvl 2 Scoring");
  }

  public Command lvl3(BooleanSupplier driveBaseAtTarget) {
    return expose(
            elevator
                .lvl3()
                .withDeadline(
                    waitUntil(() -> driveBaseAtTarget.getAsBoolean() && elevator.atTargetPos())
                        .andThen(coralOuttake.scoreFast().asProxy())
                        .withName("Score On Branch"))
                .withName("Lvl 3 Scoring"))
        .withName("Lvl 3 Scoring");
  }

  public Command lvl4(BooleanSupplier driveBaseAtTarget) {
    return expose(
            elevator
                .lvl4()
                .withDeadline(
                    waitUntil(() -> driveBaseAtTarget.getAsBoolean() && elevator.atTargetPos())
                        .andThen(coralOuttake.scoreSlow().asProxy())
                        .withName("Score On Branch"))
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

  public Command intakeLowAlgae() {
    return expose(parallel(algaeClaw.intake(), elevator.lowAlgae()).withName("Low Algae"))
        .withName("Low Algae");
  }

  public Command intakeHighAlgae() {
    return expose(parallel(algaeClaw.intake(), elevator.highAlgae()).withName("High Algae"))
        .withName("High Algae");
  }

  public Command bargeScore(BooleanSupplier atBarge) {
    return expose(
            waitUntil(algaeClaw::isPastVertical)
                .andThen(
                    elevator
                        .net()
                        .withDeadline(
                            waitUntil(
                                    () ->
                                        atBarge.getAsBoolean()
                                            && elevator.atTargetPos()
                                            && algaeClaw.atTargetAngle())
                                .andThen(algaeClaw.eject().withTimeout(.2)))
                        .withName("Barge Score")))
        .withName("Barge Score");
  }

  public Command ejectAlgae() {
    return expose(algaeClaw.eject());
  }

  private boolean climberDeployed = false;

  public boolean isClimberDeployed() {
    return climberDeployed;
  }

  public Command readyClimb() {
    return expose(
            runOnce(() -> climberDeployed = true)
                .andThen(parallel(ramp.raise(), climber.deploy()))
                .withName("Ready Climb"))
        .withName("Ready Climb");
  }

  public Command climb() {
    return expose(climber.climb().withName("Climb")).withName("Climb");
  }

  public Command raiseClimber() {
    return expose(climber.deploy()).withName("Raise Climber");
  }

  public Trigger coralStuck() {
    return new Trigger(coralOuttake::coralStuck);
  }

  public boolean hasCoral() {
    return coralOuttake.hasCoral();
  }

  public boolean hasAlgae() {
    return algaeClaw.hasAlgae();
  }

  public double getElevatorExtension() {
    return elevator.getExtension();
  }
}
