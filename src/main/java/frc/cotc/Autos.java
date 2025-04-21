// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.cotc.drive.Swerve;
import frc.cotc.superstructure.Superstructure;
import frc.cotc.util.ReefLocations;
import frc.cotc.util.ReefLocations.ReefBranch;
import java.util.HashMap;
import java.util.function.Supplier;
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

    addRoutine("1AtG", () -> scoreAtG(swerve, superstructure));
    addRoutine(
        "GAnd2Barge",
        () ->
            generateAlgaeRoutine(
                swerve,
                superstructure,
                ReefBranch.G,
                new AlgaeHandling(3, AlgaeScoring.NET),
                new AlgaeHandling(4, AlgaeScoring.NET)));
    addRoutine(
        "GAnd3Barge",
        () ->
            generateAlgaeRoutine(
                swerve,
                superstructure,
                ReefBranch.G,
                new AlgaeHandling(3, AlgaeScoring.NET),
                new AlgaeHandling(2, AlgaeScoring.NET),
                new AlgaeHandling(4, AlgaeScoring.NET)));
    addRoutine(
        "GAnd2Proc",
        () ->
            generateAlgaeRoutine(
                swerve,
                superstructure,
                ReefBranch.G,
                new AlgaeHandling(3, AlgaeScoring.PROCESS),
                new AlgaeHandling(2, AlgaeScoring.PROCESS)));
    addRoutine(
        "GAnd2ProcAnd1Net",
        () ->
            generateAlgaeRoutine(
                swerve,
                superstructure,
                ReefBranch.G,
                new AlgaeHandling(3, AlgaeScoring.NET),
                new AlgaeHandling(4, AlgaeScoring.PROCESS),
                new AlgaeHandling(2, AlgaeScoring.PROCESS)));
    addRoutine(
        "3FromE",
        () ->
            generateCoralRoutine(
                swerve, superstructure, Source.R, ReefBranch.E, ReefBranch.D, ReefBranch.C));
    addRoutine(
        "3FromJ",
        () ->
            generateCoralRoutine(
                swerve, superstructure, Source.L, ReefBranch.J, ReefBranch.K, ReefBranch.L));
    addRoutine(
        "4FromE",
        () ->
            generateCoralRoutine(
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
            generateCoralRoutine(
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

  private Command scoreAtG(Swerve swerve, Superstructure superstructure) {
    return parallel(
        reefPathfinding.goTo(ReefBranch.G),
        waitUntil(swerve::nearingTargetPose)
            .andThen(superstructure.lvl4(swerve::atTargetPoseAuto)));
  }

  private final boolean[] highAlgae = new boolean[] {true, false, true, false, true, false};

  private final Pose2d leftEndPose = new Pose2d(4, 7.25, sourceLeft.getRotation());
  private final Pose2d rightEndPose =
      new Pose2d(
          leftEndPose.getX(),
          Constants.FIELD_WIDTH_METERS - leftEndPose.getY(),
          sourceRight.getRotation());

  private enum AlgaeScoring {
    PROCESS,
    NET
  }

  private record AlgaeHandling(int face, AlgaeScoring scoring) {}

  @SuppressWarnings("SameParameterValue")
  private Command generateAlgaeRoutine(
      Swerve swerve,
      Superstructure superstructure,
      ReefBranch startingBranch,
      AlgaeHandling... algaeHandling) {
    var reefFaces =
        Robot.isOnRed() ? ReefLocations.RED_ALGAE_POSES : ReefLocations.BLUE_ALGAE_POSES;
    var targetY = Constants.FIELD_WIDTH_METERS / 2 + (Robot.isOnRed() ? -1 : 1);

    var commands = new Command[algaeHandling.length * 2 + 3];

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
    commands[1] = swerve.driveStraight(-1).withTimeout(.75).withName("Back Up").asProxy();

    for (int i = 0; i < algaeHandling.length; i++) {
      commands[i * 2 + 2] =
          waitUntil(swerve::nearingTargetPose)
              .andThen(
                  highAlgae[algaeHandling[i].face]
                      ? superstructure.intakeHighAlgae()
                      : superstructure.intakeLowAlgae())
              .withDeadline(
                  swerve
                      .followRepulsorField(reefFaces[algaeHandling[i].face])
                      .until(superstructure::hasAlgae)
                      .andThen(swerve.driveStraight(-1.25).withTimeout(.25)))
              .withName("Intake Algae At " + algaeHandling[i].face)
              .asProxy()
              .raceWith(waitUntil(swerve::atTargetPoseAuto).andThen(waitSeconds(3)))
              .withName("Intake Algae At " + algaeHandling[i].face);
      commands[i * 2 + 3] =
          switch (algaeHandling[i].scoring) {
            case NET ->
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
                    .withName("Score in net")
                    .asProxy()
                    .onlyIf(superstructure::hasAlgae);
            case PROCESS ->
                swerve
                    .processorAlign(() -> Translation2d.kZero)
                    .withDeadline(superstructure.processAlgae(swerve::atTargetPoseAuto))
                    .withName("Score in processor")
                    .asProxy()
                    .andThen(
                        sequence(
                                swerve.driveStraight(.9).withTimeout(.2),
                                swerve.driveStraight(0).withTimeout(.3),
                                swerve.driveStraight(-1).withTimeout(.3))
                            .withName("Push in algae")
                            .asProxy())
                    .onlyIf(superstructure::hasAlgae);
          };
    }

    commands[algaeHandling.length * 2 + 2] =
        either(
                swerve.followRepulsorField(
                    Robot.isOnRed()
                        ? leftEndPose.rotateAround(Constants.FIELD_CENTER, Rotation2d.kPi)
                        : leftEndPose),
                swerve.followRepulsorField(
                    Robot.isOnRed()
                        ? rightEndPose.rotateAround(Constants.FIELD_CENTER, Rotation2d.kPi)
                        : rightEndPose),
                () ->
                    Robot.isOnRed()
                        ? swerve.getPose().getY() < Constants.FIELD_WIDTH_METERS / 2
                        : swerve.getPose().getY() > Constants.FIELD_WIDTH_METERS / 2)
            .until(swerve::atTargetPoseAuto)
            .withName("Run to source")
            .asProxy();

    return sequence(commands);
  }

  private Command generateCoralRoutine(
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
}
