// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.cotc.drive.Swerve;
import frc.cotc.superstructure.Superstructure;
import frc.cotc.util.ReefLocations;
import frc.cotc.util.ReefLocations.ReefBranch;
import java.util.HashMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {
  private final LoggedDashboardChooser<String> chooser;

  private final HashMap<String, Supplier<Command>> routines = new HashMap<>();
  private final String NONE_NAME = "Do Nothing";

  @FunctionalInterface
  private interface ReefRepulsorCommand {
    Command goTo(ReefBranch branch);
  }

  @FunctionalInterface
  private interface SourceRepulsorCommand {
    Command goTo(Source source);
  }

  private final ReefRepulsorCommand reefPathfinding;
  private final SourceRepulsorCommand sourcePathfinding;

  public Autos(Swerve swerve, Superstructure superstructure) {
    chooser = new LoggedDashboardChooser<>("Auto Chooser");
    chooser.addDefaultOption(NONE_NAME, NONE_NAME);
    routines.put(NONE_NAME, Commands::none);

    var factory =
        new AutoFactory(
            swerve::getPose,
            swerve::resetForAuto,
            swerve::followChoreoTrajectory,
            true,
            swerve,
            (trajectory, starting) -> {
              var poses = trajectory.getPoses();
              if (Robot.isOnRed()) {
                for (int i = 0; i < poses.length; i++) {
                  poses[i] =
                      new Pose2d(
                          poses[i]
                              .getTranslation()
                              .rotateAround(Constants.FIELD_CENTER, Rotation2d.kPi),
                          poses[i].getRotation().rotateBy(Rotation2d.kPi));
                }
              }
              Logger.recordOutput("Choreo/Trajectory", poses);
            });
    reefPathfinding =
        branch ->
            swerve
                .followRepulsorField(ReefLocations.getScoringLocation(branch))
                .withName("Reef Align " + branch.name());
    sourcePathfinding =
        source -> {
          var pose = source == Source.L ? sourceLeft : sourceRight;
          if (Robot.isOnRed()) {
            pose = pose.rotateAround(Constants.FIELD_CENTER, Rotation2d.kPi);
          }
          return swerve.followRepulsorField(pose).withName("Source Align " + source.name());
        };

    addRoutine("1AtG", () -> scoreAtG(factory, swerve, superstructure));
    addRoutine(
        "GAnd2Algae", () -> generateAlgaeRoutine(swerve, superstructure, ReefBranch.G, 3, 4));
    addRoutine(
        "GAnd3Algae", () -> generateAlgaeRoutine(swerve, superstructure, ReefBranch.G, 3, 2, 4));
    addRoutine(
        "3FromE",
        () ->
            generateChoreoRoutine(
                factory,
                swerve,
                superstructure,
                Source.R,
                ReefBranch.E,
                ReefBranch.D,
                ReefBranch.C));
    addRoutine(
        "3FromJ",
        () ->
            generateChoreoRoutine(
                factory,
                swerve,
                superstructure,
                Source.L,
                ReefBranch.J,
                ReefBranch.K,
                ReefBranch.L));
    addRoutine(
        "Fast3FromE",
        () ->
            generateRepulsorRoutine(
                swerve, superstructure, Source.R, ReefBranch.E, ReefBranch.D, ReefBranch.C));
    addRoutine(
        "Fast3FromJ",
        () ->
            generateRepulsorRoutine(
                swerve, superstructure, Source.L, ReefBranch.J, ReefBranch.K, ReefBranch.L));
    addRoutine(
        "4FromE",
        () ->
            generateRepulsorRoutine(
                swerve,
                superstructure,
                Source.R,
                ReefBranch.E,
                ReefBranch.D,
                ReefBranch.C,
                ReefBranch.B));
    addRoutine(
        "4FromJ",
        () ->
            generateRepulsorRoutine(
                swerve,
                superstructure,
                Source.L,
                ReefBranch.J,
                ReefBranch.K,
                ReefBranch.L,
                ReefBranch.A));
  }

  private final Pose2d sourceRight = new Pose2d(1.61, .67, Rotation2d.fromDegrees(54));
  private final Pose2d sourceLeft =
      new Pose2d(
          sourceRight.getX(),
          Constants.FIELD_WIDTH_METERS - sourceRight.getY(),
          sourceRight.getRotation().unaryMinus());

  private Command scoreAtG(AutoFactory factory, Swerve swerve, Superstructure superstructure) {
    var routine = factory.newRoutine("ScoreAtG");

    routine
        .active()
        .onTrue(
            parallel(
                reefPathfinding.goTo(ReefBranch.G),
                waitUntil(swerve::nearingTargetPose)
                    .andThen(superstructure.lvl4(swerve::atTargetPoseAuto))));

    return routine.cmd();
  }

  private Command generateAlgaeRoutine(
      Swerve swerve, Superstructure superstructure, ReefBranch startingBranch, int... faces) {
    var reefFaces =
        Robot.isOnRed() ? ReefLocations.RED_ALGAE_POSES : ReefLocations.BLUE_ALGAE_POSES;
    var targetY = Constants.FIELD_WIDTH_METERS / 2 + (Robot.isOnRed() ? -1 : 1);

    var high = new boolean[] {true, false, true, false, true, false};

    var commands = new Command[faces.length * 2 + 3];

    commands[0] =
        reefPathfinding
            .goTo(startingBranch)
            .withDeadline(
                waitUntil(swerve::nearingTargetPose)
                    .withName("Wait for drivebase")
                    .andThen(superstructure.lvl4(swerve::atTargetPoseAuto))
                    .withName("Score L4"))
            .withName("Go to " + startingBranch.name())
            .asProxy();
    commands[1] = swerve.driveStraight(-.75).withTimeout(.5).asProxy();

    for (int i = 0; i < faces.length; i++) {
      commands[i * 2 + 2] =
          waitUntil(swerve::nearingTargetPose)
              .andThen(
                  high[faces[i]]
                      ? superstructure.intakeHighAlgae()
                      : superstructure.intakeLowAlgae())
              .withDeadline(
                  swerve
                      .followRepulsorField(reefFaces[faces[i]])
                      .until(superstructure::hasAlgae)
                      .andThen(swerve.driveStraight(-1.25).withTimeout(.25)))
              .asProxy()
              .raceWith(waitUntil(swerve::atTargetPoseAuto).andThen(waitSeconds(3)));
      commands[i * 2 + 3] =
          swerve
              .netAlign(
                  () ->
                      new Translation2d(
                          0,
                          2
                              * (Robot.isOnRed()
                                  ? swerve.getPose().getY() - targetY
                                  : targetY - swerve.getPose().getY())))
              .withDeadline(
                  waitUntil(swerve::nearingNet)
                      .andThen(superstructure.bargeScore(swerve::atNet).asProxy()))
              .asProxy()
              .onlyIf(superstructure::hasAlgae);
    }

    var leftEndPose = new Pose2d(4, 7.25, sourceLeft.getRotation());
    if (Robot.isOnRed()) {
      leftEndPose = leftEndPose.rotateAround(Constants.FIELD_CENTER, Rotation2d.kPi);
    }
    var rightEndPose =
        new Pose2d(4, Constants.FIELD_WIDTH_METERS - 7.25, sourceRight.getRotation());
    if (Robot.isOnRed()) {
      rightEndPose = rightEndPose.rotateAround(Constants.FIELD_CENTER, Rotation2d.kPi);
    }
    commands[faces.length * 2 + 2] =
        either(
                swerve.followRepulsorField(leftEndPose),
                swerve.followRepulsorField(rightEndPose),
                () ->
                    Robot.isOnRed()
                        ? swerve.getPose().getY() < Constants.FIELD_WIDTH_METERS / 2
                        : swerve.getPose().getY() > Constants.FIELD_WIDTH_METERS / 2)
            .asProxy();

    return sequence(commands);
  }

  private Command generateChoreoRoutine(
      AutoFactory factory,
      Swerve swerve,
      Superstructure superstructure,
      Source source,
      ReefBranch initialBranch,
      ReefBranch... cyclingBranches) {
    var routine = factory.newRoutine("Autogenerated Routine");

    var reefToSource = new AutoTrajectory[cyclingBranches.length];
    var sourceToReef = new AutoTrajectory[cyclingBranches.length];
    reefToSource[0] = getTrajectory(routine, initialBranch, source);
    sourceToReef[0] = getTrajectory(routine, source, cyclingBranches[0]);
    for (int i = 1; i < cyclingBranches.length; i++) {
      reefToSource[i] = getTrajectory(routine, cyclingBranches[i - 1], source);
      sourceToReef[i] = getTrajectory(routine, source, cyclingBranches[i]);
    }
    var nextCycleSpwnCmd = new Command[sourceToReef.length];
    for (int i = 0; i < sourceToReef.length - 1; i++) {
      nextCycleSpwnCmd[i] = reefToSource[i + 1].spawnCmd();
    }
    nextCycleSpwnCmd[sourceToReef.length - 1] =
        new ScheduleCommand(
            getTrajectory(routine, cyclingBranches[cyclingBranches.length - 1], source)
                .cmd()
                .andThen(sourcePathfinding.goTo(source)));

    routine
        .active()
        .onTrue(
            sequence(
                    waitUntil(swerve::nearingTargetPose),
                    superstructure.lvl4(swerve::atTargetPoseAuto))
                .deadlineFor(reefPathfinding.goTo(initialBranch))
                .withName("ScoreAt" + initialBranch.name())
                .andThen(reefToSource[0].spawnCmd())
                .withName("StartTo" + initialBranch.name()));

    for (int i = 0; i < cyclingBranches.length; i++) {
      reefToSource[i]
          .done()
          .onTrue(
              waitUntil(superstructure::hasCoral)
                  // .withTimeout(.5) // ONLY RUN IN SIM
                  .deadlineFor(sourcePathfinding.goTo(source))
                  .andThen(sourceToReef[i].spawnCmd())
                  .withName("Source" + source.name()));

      sourceToReef[i]
          .atTimeBeforeEnd(.5)
          .onTrue(
              superstructure
                  .lvl4(swerve::atTargetPoseAuto)
                  .deadlineFor(
                      waitUntil(sourceToReef[i].done())
                          .andThen(reefPathfinding.goTo(cyclingBranches[i]).asProxy()))
                  .andThen(nextCycleSpwnCmd[i])
                  .withName("ScoreAt" + cyclingBranches[i].name()));
    }

    return routine.cmd();
  }

  private Command generateRepulsorRoutine(
      Swerve swerve, Superstructure superstructure, Source source, ReefBranch... reefBranches) {
    var commands = new Command[reefBranches.length * 2];

    for (int i = 0; i < reefBranches.length; i++) {
      commands[i * 2] =
          reefPathfinding
              .goTo(reefBranches[i])
              .withDeadline(
                  waitUntil(swerve::nearingTargetPose)
                      .withName("Wait for drivebase")
                      .andThen(superstructure.lvl4(swerve::atTargetPoseAuto))
                      .withName("Score L4"))
              .withName("Go to " + reefBranches[i].name())
              .asProxy();
      commands[i * 2 + 1] =
          sourcePathfinding
              .goTo(source)
              .until(superstructure::hasCoral)
              .withName("Go to intake")
              .asProxy();
    }

    return sequence(commands);
  }

  private String selectedCommandName = NONE_NAME;
  private Command selectedCommand = none();
  private boolean selectedOnRed = false;

  private final Alert selectedNonexistentAuto =
      new Alert("Selected an auto that isn't an option!", Alert.AlertType.kError);
  private final Alert loadedAutoAlert = new Alert("", Alert.AlertType.kInfo);

  public void update() {
    if (DriverStation.isDSAttached() && DriverStation.getAlliance().isPresent()) {
      var selected = chooser.get();
      if (selected.equals(selectedCommandName) && selectedOnRed == Robot.isOnRed()) {
        return;
      }
      if (!routines.containsKey(selected)) {
        selected = NONE_NAME;
        selectedNonexistentAuto.set(true);
      } else {
        selectedNonexistentAuto.set(false);
      }
      selectedCommandName = selected;
      selectedCommand = routines.get(selected).get().withName(selectedCommandName);
      selectedOnRed = Robot.isOnRed();
      loadedAutoAlert.setText("Loaded Auto: " + selectedCommandName);
      loadedAutoAlert.set(true);
    }
  }

  public void clear() {
    selectedCommandName = NONE_NAME;
    selectedCommand = none();
    selectedOnRed = false;
  }

  public Command getSelectedCommand() {
    return selectedCommand;
  }

  private void addRoutine(String name, Supplier<Command> generator) {
    chooser.addOption(name, name);
    routines.put(name, generator);
  }

  private enum Source {
    L,
    R
  }

  private AutoTrajectory getTrajectory(AutoRoutine routine, ReefBranch reefBranch, Source source) {
    return routine.trajectory(reefBranch.name() + "~S" + source.name(), 0);
  }

  private AutoTrajectory getTrajectory(AutoRoutine routine, Source source, ReefBranch reefBranch) {
    return routine.trajectory(reefBranch.name() + "~S" + source.name(), 1);
  }
}
