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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.cotc.drive.Swerve;
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
            swerve::followTrajectory,
            true,
            swerve,
            new AutoFactory.AutoBindings(),
            (trajectory, starting) -> {
              var poses = trajectory.getPoses();
              if (Robot.isOnRed()) {
                for (int i = 0; i < poses.length; i++) {
                  poses[i] =
                      new Pose2d(16.54 - poses[i].getX(), poses[i].getY(), poses[i].getRotation());
                }
              }
              Logger.recordOutput("Swerve/Trajectory", poses);
            });

    addRoutine("Four Note Auto", () -> fourNoteAuto(factory));
  }

  private AutoRoutine fourNoteAuto(AutoFactory factory) {
    final var routine = factory.newRoutine("Four Note Auto");

    final var speakerToC1 = routine.trajectory("speakerToC1");
    final var C1ToSpeaker = routine.trajectory("C1ToSpeaker");
    final var C1ToC2 = routine.trajectory("C1ToC2");
    final var speakerToC2 = routine.trajectory("speakerToC2");
    final var C2ToSpeaker = routine.trajectory("C2ToSpeaker");
    final var C2ToC3 = routine.trajectory("C2ToC3");
    final var speakerToC3 = routine.trajectory("speakerToC3");
    final var C3ToSpeaker = routine.trajectory("C3ToSpeaker");

    // Score and then go to C1
    routine.active().onTrue(routine.resetOdometry(speakerToC1).andThen(speakerToC1.cmd()));

    // Go score if it has C1, skip to C2 if it doesn't
    speakerToC1.done().and(hasGamePiece(routine)).onTrue(C1ToSpeaker.cmd());
    speakerToC1.done().and(noGamePiece(routine)).onTrue(C1ToC2.cmd());

    // Once it scores C1, go to C2
    C1ToSpeaker.done().onTrue(speakerToC2.cmd());

    // Go score if it has C2, skip to C3 if it doesn't
    speakerToC2.done().and(hasGamePiece(routine)).onTrue(C2ToSpeaker.cmd());
    speakerToC2.done().and(noGamePiece(routine)).onTrue(C2ToC3.cmd());
    C1ToC2.done().and(hasGamePiece(routine)).onTrue(C2ToSpeaker.cmd());
    C1ToC2.done().and(noGamePiece(routine)).onTrue(C2ToC3.cmd());

    // Once it scores C2, go to C3
    C2ToSpeaker.done().onTrue(speakerToC3.cmd());

    // Go score regardless of status
    speakerToC3.done().onTrue(C3ToSpeaker.cmd());
    C2ToC3.done().onTrue(C3ToSpeaker.cmd());

    return routine;
  }

  private Trigger hasGamePiece(AutoRoutine routine) {
    return routine.observe(() -> true);
  }

  private Trigger noGamePiece(AutoRoutine routine) {
    return routine.observe(() -> false);
  }

  private String selectedCommandName = NONE_NAME;
  private Command selectedCommand = none();

  private final Alert selectedNonexistentAuto =
      new Alert("Selected an auto that isn't an option!", Alert.AlertType.kError);

  public void update() {
    if (DriverStation.isDSAttached() && DriverStation.getAlliance().isPresent()) {
      var selected = chooser.get();
      if (selected.equals(selectedCommandName)) {
        return;
      }
      if (!routines.containsKey(selected)) {
        selected = NONE_NAME;
        selectedNonexistentAuto.set(true);
      } else {
        selectedNonexistentAuto.set(false);
      }
      selectedCommandName = selected;
      selectedCommand = routines.get(selected).get();
    }
  }

  public Command getSelectedCommand() {
    return selectedCommand.withName(selectedCommandName);
  }

  private void addRoutine(String name, Supplier<AutoRoutine> generator) {
    chooser.addOption(name, name);
    routines.put(name, () -> generator.get().cmd());
  }
}
