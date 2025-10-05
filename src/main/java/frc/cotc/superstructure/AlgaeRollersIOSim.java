// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.*;
import frc.cotc.Constants;
import frc.cotc.Robot;
import frc.cotc.util.ReefLocations;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class AlgaeRollersIOSim implements AlgaeRollersIO {
  private boolean hasAlgae = false;

  private final Pose3d lowAlgaeBasePose =
      new Pose3d(
          new Translation3d(3.805, Constants.FIELD_WIDTH_METERS / 2, 1.315), Rotation3d.kZero);
  private final Pose3d highAlgaeBasePose =
      lowAlgaeBasePose.transformBy(new Transform3d(0, 0, -.4, Rotation3d.kZero));

  DoubleSupplier elevatorHeight;

  @Override
  public void updateInputs(AlgaeRollersIOInputs inputs) {
    inputs.hasAlgae = hasAlgae;

    var reefAlgae = new ArrayList<Pose3d>();
    for (int i = 0; i < 12; i++) {
      if (reefHasAlgae[i]) {
        Pose3d baseAlgaePose;
        if (i % 2 == 0) {
          baseAlgaePose = lowAlgaeBasePose;
        } else {
          baseAlgaePose = highAlgaeBasePose;
        }

        Translation2d reefCenter;
        if (i < 6) {
          reefCenter = ReefLocations.BLUE_REEF;
        } else {
          baseAlgaePose =
              baseAlgaePose.rotateAround(
                  new Translation3d(Constants.FIELD_CENTER), new Rotation3d(0, 0, Math.PI));
          reefCenter = ReefLocations.RED_REEF;
        }

        reefAlgae.add(
            baseAlgaePose.rotateAround(
                new Translation3d(reefCenter), new Rotation3d(0, 0, i * Math.PI / 3)));
      }
    }
    Logger.recordOutput("Sim/Reef Algae", reefAlgae.toArray(new Pose3d[0]));

    var bargeAlgae = new ArrayList<Pose3d>();
    for (int i = 0; i < blueBargeAlgaeCount; i++) {
      bargeAlgae.add(
          new Pose3d(
              new Translation3d(
                  Constants.FIELD_LENGTH_METERS / 2,
                  Constants.FIELD_WIDTH_METERS - (.5 + i * .45),
                  2.1),
              Rotation3d.kZero));
    }
    for (int i = 0; i < redBargeAlgaeCount; i++) {
      bargeAlgae.add(
          new Pose3d(
              new Translation3d(Constants.FIELD_LENGTH_METERS / 2, .5 + i * .4, 2.1),
              Rotation3d.kZero));
    }
    Logger.recordOutput("Sim/Barge Algae", bargeAlgae.toArray(new Pose3d[0]));

    if (hasAlgae) {
      Logger.recordOutput(
          "Sim/Robot Algae",
          new Pose3d(Robot.groundTruthPoseSupplier.get())
              .plus(
                  new Transform3d(
                      new Translation3d(-.15, 0, 1.2 + elevatorHeight.getAsDouble()),
                      Rotation3d.kZero)));
    } else {
      Logger.recordOutput("Sim/Robot Algae", new Pose3d(0, 0, -100, Rotation3d.kZero));
    }
  }

  BooleanSupplier atTargetAngle;
  BooleanSupplier atTargetHeight;

  Debouncer debouncer = new Debouncer(.2);

  private final boolean[] reefHasAlgae =
      new boolean[] {true, true, true, true, true, true, true, true, true, true, true, true};

  @Override
  public void intake() {
    if (!hasAlgae) {
      var groundTruthPose = Robot.groundTruthPoseSupplier.get();
      var groundTruthSpeed = Robot.groundTruthSpeedSupplier.get();

      int closestReefAlgaeSpot = -1;
      double closestReefAlgaeDistance = Double.POSITIVE_INFINITY;
      for (int i = 0; i < 6; i++) {
        double blueDistance =
            ReefLocations.BLUE_ALGAE_POSES[i]
                .getTranslation()
                .getDistance(groundTruthPose.getTranslation());
        if (blueDistance < closestReefAlgaeDistance) {
          closestReefAlgaeSpot = i;
          closestReefAlgaeDistance = blueDistance;
        }
        double redDistance =
            ReefLocations.RED_ALGAE_POSES[i]
                .getTranslation()
                .getDistance(groundTruthPose.getTranslation());
        if (redDistance < closestReefAlgaeDistance) {
          closestReefAlgaeSpot = i + 6;
          closestReefAlgaeDistance = redDistance;
        }
      }

      if (!reefHasAlgae[closestReefAlgaeSpot]) {
        return;
      }

      var algaePosition =
          closestReefAlgaeSpot < 6
              ? ReefLocations.BLUE_ALGAE_POSES[closestReefAlgaeSpot]
              : ReefLocations.RED_ALGAE_POSES[closestReefAlgaeSpot - 6];
      var positionDelta = algaePosition.minus(groundTruthPose);

      hasAlgae =
          debouncer.calculate(
              positionDelta.getTranslation().getNorm() < .25
                  && Math.abs(positionDelta.getRotation().getDegrees()) < 10
                  && Math.hypot(
                          groundTruthSpeed.vxMetersPerSecond, groundTruthSpeed.vyMetersPerSecond)
                      < .1
                  && atTargetAngle.getAsBoolean()
                  && atTargetHeight.getAsBoolean());
      if (hasAlgae) {
        reefHasAlgae[closestReefAlgaeSpot] = false;
      }
    }
  }

  private int blueBargeAlgaeCount = 0;
  private int redBargeAlgaeCount = 0;

  @Override
  public void eject() {
    hasAlgae = false;
    var robotPose = Robot.groundTruthPoseSupplier.get();
    if (MathUtil.isNear(robotPose.getX(), 7.8, .5)
        || MathUtil.isNear(robotPose.getX(), Constants.FIELD_LENGTH_METERS - 7.8, .5)) {
      if (robotPose.getY() > Constants.FIELD_WIDTH_METERS / 2) {
        blueBargeAlgaeCount++;
      } else {
        redBargeAlgaeCount++;
      }
    }
  }
}
