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
import frc.cotc.util.ReefLocations.ReefBranch;
import java.util.HashMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {
  private final LoggedDashboardChooser<String> chooser;

  private final HashMap<String, Supplier<Command>> routines = new HashMap<>();
  private final String NONE_NAME = "Do Nothing";

  public Autos(Swerve swerve) {
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

    addRoutine("ScoreOne", () -> scoreOne(factory));
  }

  private AutoRoutine scoreOne(AutoFactory factory) {
    var routine = factory.newRoutine("scoreOne");

    var startToG = routine.trajectory("StartToG");
    var gToSource = getTrajectory(routine, ReefBranch.G, SourceLoc.R);
    var sourceToC = getTrajectory(routine, SourceLoc.R, ReefBranch.C);
    var cToSource = getTrajectory(routine, ReefBranch.C, SourceLoc.R);
    var sourceToD = getTrajectory(routine, SourceLoc.R, ReefBranch.D);

    routine.active().onTrue(startToG.resetOdometry().andThen(startToG.cmd()));

    startToG.done().onTrue(gToSource.cmd());
    gToSource.done().onTrue(sourceToC.cmd());
    sourceToC.done().onTrue(cToSource.cmd());
    cToSource.done().onTrue(sourceToD.cmd());

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

  private enum SourceLoc {
    L,
    R
  }

  @SuppressWarnings("SameParameterValue")
  private AutoTrajectory getTrajectory(
      AutoRoutine routine, ReefBranch reefBranch, SourceLoc sourceLoc) {
    return routine.trajectory(reefBranch.name() + "~S" + sourceLoc.name(), 0);
  }

  @SuppressWarnings("SameParameterValue")
  private AutoTrajectory getTrajectory(
      AutoRoutine routine, SourceLoc sourceLoc, ReefBranch reefBranch) {
    return routine.trajectory(reefBranch.name() + "~S" + sourceLoc.name(), 1);
  }
}
