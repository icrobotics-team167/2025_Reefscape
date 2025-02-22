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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.cotc.drive.Swerve;
import frc.cotc.superstructure.Superstructure;
import frc.cotc.util.ReefLocations;
import frc.cotc.util.ReefLocations.ReefBranch;
import java.util.HashMap;
import java.util.Set;
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

  private final ReefRepulsorCommand reefRepulsorCommand;
  private final SourceRepulsorCommand sourceRepulsorCommand;

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
    reefRepulsorCommand =
        branch -> swerve.followRepulsorField(ReefLocations.getScoringLocation(branch));
    sourceRepulsorCommand =
        source ->
            defer(
                () -> {
                  var pose = source == Source.L ? sourceLeft : sourceRight;
                  if (Robot.isOnRed()) {
                    pose.rotateAround(Constants.FIELD_CENTER, Rotation2d.kPi);
                  }
                  return swerve.followRepulsorField(pose);
                },
                Set.of(swerve));

    addRoutine("CycleFromE", () -> cycleFromE(factory, swerve, superstructure));
    addRoutine("CycleFromJ", () -> cycleFromJ(factory, swerve, superstructure));
    addRoutine("L&K", () -> lAndK(factory, swerve, superstructure));
    addRoutine("DriveTest", () -> driveTest(factory, swerve));
  }

  private final Pose2d sourceRight = new Pose2d(1.61, .6, Rotation2d.fromDegrees(54));
  private final Pose2d sourceLeft =
      new Pose2d(1.61, Constants.FIELD_WIDTH_METERS - .6, Rotation2d.fromDegrees(-54));

  private AutoRoutine cycleFromE(
      AutoFactory factory, Swerve swerve, Superstructure superstructure) {
    var routine = factory.newRoutine("CycleFromE");

    var eToSource = getTrajectory(routine, ReefBranch.E, Source.R);
    var sourceToC = getTrajectory(routine, Source.R, ReefBranch.C);
    var cToSource = getTrajectory(routine, ReefBranch.C, Source.R);
    var sourceToD = getTrajectory(routine, Source.R, ReefBranch.D);
    var dToSource = getTrajectory(routine, ReefBranch.D, Source.R);
    var sourceToB = getTrajectory(routine, Source.R, ReefBranch.B);
    var bToSource = getTrajectory(routine, ReefBranch.B, Source.R);

    routine
        .active()
        .onTrue(
            deadline(
                    superstructure.lvl4(swerve::atTargetPose),
                    reefRepulsorCommand.goTo(ReefBranch.E))
                .andThen(eToSource.spawnCmd()));

    eToSource
        .done()
        .onTrue(
            sequence(
                superstructure
                    .intake()
                    .withTimeout(1)
                    .deadlineFor(sourceRepulsorCommand.goTo(Source.R)),
                superstructure
                    .lvl4(swerve::atTargetPose)
                    .deadlineFor(sourceToD.cmd().andThen(reefRepulsorCommand.goTo(ReefBranch.D)))
                    .andThen(dToSource.spawnCmd())));

    dToSource
        .done()
        .onTrue(
            sequence(
                superstructure
                    .intake()
                    .withTimeout(1)
                    .deadlineFor(sourceRepulsorCommand.goTo(Source.R)),
                superstructure
                    .lvl4(swerve::atTargetPose)
                    .deadlineFor(sourceToC.cmd().andThen(reefRepulsorCommand.goTo(ReefBranch.C)))
                    .andThen(cToSource.spawnCmd())));

    cToSource
        .done()
        .onTrue(
            sequence(
                superstructure
                    .intake()
                    .withTimeout(1)
                    .deadlineFor(sourceRepulsorCommand.goTo(Source.R)),
                superstructure
                    .lvl4(swerve::atTargetPose)
                    .deadlineFor(sourceToB.cmd().andThen(reefRepulsorCommand.goTo(ReefBranch.B)))
                    .andThen(bToSource.spawnCmd())));

    return routine;
  }

  private AutoRoutine cycleFromJ(
      AutoFactory factory, Swerve swerve, Superstructure superstructure) {
    var routine = factory.newRoutine("CycleFromJ");

    var jToSource = getTrajectory(routine, ReefBranch.J, Source.L);
    var sourceToL = getTrajectory(routine, Source.L, ReefBranch.L);
    var lToSource = getTrajectory(routine, ReefBranch.L, Source.L);
    var sourceToK = getTrajectory(routine, Source.L, ReefBranch.K);
    var kToSource = getTrajectory(routine, ReefBranch.K, Source.L);
    var sourceToA = getTrajectory(routine, Source.L, ReefBranch.A);
    var aToSource = getTrajectory(routine, ReefBranch.A, Source.L);

    routine
        .active()
        .onTrue(
            deadline(
                    superstructure.lvl4(swerve::atTargetPose),
                    reefRepulsorCommand.goTo(ReefBranch.J))
                .andThen(jToSource.spawnCmd()));

    jToSource
        .done()
        .onTrue(
            sequence(
                superstructure
                    .intake()
                    .withTimeout(1)
                    .deadlineFor(sourceRepulsorCommand.goTo(Source.L)),
                superstructure
                    .lvl4(swerve::atTargetPose)
                    .deadlineFor(sourceToK.cmd().andThen(reefRepulsorCommand.goTo(ReefBranch.K)))
                    .andThen(kToSource.spawnCmd())));

    kToSource
        .done()
        .onTrue(
            sequence(
                superstructure
                    .intake()
                    .withTimeout(1)
                    .deadlineFor(sourceRepulsorCommand.goTo(Source.L)),
                superstructure
                    .lvl4(swerve::atTargetPose)
                    .deadlineFor(sourceToL.cmd().andThen(reefRepulsorCommand.goTo(ReefBranch.L)))
                    .andThen(lToSource.spawnCmd())));

    lToSource
        .done()
        .onTrue(
            sequence(
                superstructure
                    .intake()
                    .withTimeout(1)
                    .deadlineFor(sourceRepulsorCommand.goTo(Source.L)),
                superstructure
                    .lvl4(swerve::atTargetPose)
                    .deadlineFor(sourceToA.cmd().andThen(reefRepulsorCommand.goTo(ReefBranch.A)))
                    .andThen(aToSource.spawnCmd())));

    return routine;
  }

  private AutoRoutine lAndK(AutoFactory factory, Swerve swerve, Superstructure superstructure) {
    var routine = factory.newRoutine("lAndK");

    routine
        .active()
        .onTrue(
            superstructure
                .lvl4(swerve::atTargetPose)
                .deadlineFor(reefRepulsorCommand.goTo(ReefBranch.L)));

    return routine;
  }

  private AutoRoutine driveTest(AutoFactory factory, Swerve swerve) {
    var routine = factory.newRoutine("driveTest");

    routine
        .active()
        .onTrue(
            sequence(
                reefRepulsorCommand.goTo(ReefBranch.J).until(swerve::atTargetPose),
                sourceRepulsorCommand.goTo(Source.L).until(swerve::atTargetPose),
                reefRepulsorCommand.goTo(ReefBranch.K).until(swerve::atTargetPose),
                sourceRepulsorCommand.goTo(Source.L).until(swerve::atTargetPose)));

    return routine;
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

  private void addRoutine(String name, Supplier<AutoRoutine> generator) {
    chooser.addOption(name, name);
    routines.put(name, () -> generator.get().cmd());
  }

  private enum Source {
    L,
    R
  }

  @SuppressWarnings("SameParameterValue")
  private AutoTrajectory getTrajectory(AutoRoutine routine, ReefBranch reefBranch, Source source) {
    return routine.trajectory(reefBranch.name() + "~S" + source.name(), 0);
  }

  @SuppressWarnings("SameParameterValue")
  private AutoTrajectory getTrajectory(AutoRoutine routine, Source source, ReefBranch reefBranch) {
    return routine.trajectory(reefBranch.name() + "~S" + source.name(), 1);
  }
}
