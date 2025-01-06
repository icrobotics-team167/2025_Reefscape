// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.cotc.drive.Swerve;
import java.util.HashMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {
  private final LoggedDashboardChooser<String> chooser;

  private final HashMap<String, Supplier<Command>> routines = new HashMap<>();
  private final String NONE_NAME = "Do Nothing";

  public Autos(Swerve swerve) {
    chooser = new LoggedDashboardChooser<>("Auto Chooser");
    chooser.addDefaultOption(NONE_NAME, NONE_NAME);
    routines.put(NONE_NAME, Commands::none);

    //    var factory =
    //        new AutoFactory(
    //            swerve::getPose,
    //            swerve::resetForAuto,
    //            swerve::followTrajectory,
    //            true,
    //            swerve,
    //            new AutoFactory.AutoBindings(),
    //            (trajectory, starting) -> {
    //              var poses = trajectory.getPoses();
    //              if (Robot.isOnRed()) {
    //                for (int i = 0; i < poses.length; i++) {
    //                  poses[i] =
    //                      new Pose2d(16.54 - poses[i].getX(), poses[i].getY(),
    // poses[i].getRotation());
    //                }
    //              }
    //              Logger.recordOutput("Swerve/Trajectory", poses);
    //            });
    //
    //    addRoutine("Four Note Auto", () -> fourNoteAuto(factory));
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

  //  private void addRoutine(String name, Supplier<AutoRoutine> generator) {
  //    chooser.addOption(name, name);
  //    routines.put(name, () -> generator.get().cmd());
  //  }
}
