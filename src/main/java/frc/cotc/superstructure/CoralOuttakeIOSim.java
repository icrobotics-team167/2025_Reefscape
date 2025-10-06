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
import edu.wpi.first.math.util.Units;
import frc.cotc.Constants;
import frc.cotc.Robot;
import frc.cotc.util.ReefLocations;
import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class CoralOuttakeIOSim implements CoralOuttakeIO {
  DoubleSupplier elevatorHeight;

  private final Pose3d[] l2BluePoses = new Pose3d[12];
  private final Pose3d[] l3BluePoses = new Pose3d[12];
  private final Pose3d[] l4BluePoses = new Pose3d[12];
  private final Pose3d[] l2RedPoses = new Pose3d[12];
  private final Pose3d[] l3RedPoses = new Pose3d[12];
  private final Pose3d[] l4RedPoses = new Pose3d[12];

  public CoralOuttakeIOSim() {
    // Base positions have the right z and x for a blue A and B branch, but y is in the center of
    // the field.
    var l2BaseCoralPos =
        new Pose3d(
            3.8,
            Constants.FIELD_WIDTH_METERS / 2,
            0.7,
            new Rotation3d(0, Units.degreesToRadians(35), 0));
    var l3BaseCoralPos =
        new Pose3d(
            3.8,
            Constants.FIELD_WIDTH_METERS / 2,
            1.1,
            new Rotation3d(0, Units.degreesToRadians(35), 0));
    var l4BaseCoralPos =
        new Pose3d(
            3.705, Constants.FIELD_WIDTH_METERS / 2, 1.75, new Rotation3d(0, Math.PI / 2, 0));
    // How far to adjust the y by to align the side-to-side correct
    double basePosSideAdjust = .165;

    // Define the poses for each level on the A and B branches
    l2BluePoses[0] =
        l2BaseCoralPos.plus(new Transform3d(0, basePosSideAdjust, 0, Rotation3d.kZero));
    l2BluePoses[1] =
        l2BaseCoralPos.plus(new Transform3d(0, -basePosSideAdjust, 0, Rotation3d.kZero));
    l3BluePoses[0] =
        l3BaseCoralPos.plus(new Transform3d(0, basePosSideAdjust, 0, Rotation3d.kZero));
    l3BluePoses[1] =
        l3BaseCoralPos.plus(new Transform3d(0, -basePosSideAdjust, 0, Rotation3d.kZero));
    l4BluePoses[0] =
        l4BaseCoralPos.plus(new Transform3d(0, basePosSideAdjust, 0, Rotation3d.kZero));
    l4BluePoses[1] =
        l4BaseCoralPos.plus(new Transform3d(0, -basePosSideAdjust, 0, Rotation3d.kZero));
    // The rest of the branches are a transformation of the A and B poses around the center of
    // the reef
    for (int i = 1; i < 6; i++) {
      var rotation = new Rotation3d(0, 0, i * Math.PI / 3);
      l2BluePoses[i * 2] =
          l2BluePoses[0].rotateAround(new Translation3d(ReefLocations.BLUE_REEF), rotation);
      l2BluePoses[i * 2 + 1] =
          l2BluePoses[1].rotateAround(new Translation3d(ReefLocations.BLUE_REEF), rotation);
      l3BluePoses[i * 2] =
          l3BluePoses[0].rotateAround(new Translation3d(ReefLocations.BLUE_REEF), rotation);
      l3BluePoses[i * 2 + 1] =
          l3BluePoses[1].rotateAround(new Translation3d(ReefLocations.BLUE_REEF), rotation);
      l4BluePoses[i * 2] =
          l4BluePoses[0].rotateAround(new Translation3d(ReefLocations.BLUE_REEF), rotation);
      l4BluePoses[i * 2 + 1] =
          l4BluePoses[1].rotateAround(new Translation3d(ReefLocations.BLUE_REEF), rotation);
    }
    // The red reef poses are a transformation around the center of the field
    for (int i = 0; i < 12; i++) {
      l2RedPoses[i] =
          l2BluePoses[i].rotateAround(
              new Translation3d(Constants.FIELD_CENTER), new Rotation3d(0, 0, Math.PI));
      l3RedPoses[i] =
          l3BluePoses[i].rotateAround(
              new Translation3d(Constants.FIELD_CENTER), new Rotation3d(0, 0, Math.PI));
      l4RedPoses[i] =
          l4BluePoses[i].rotateAround(
              new Translation3d(Constants.FIELD_CENTER), new Rotation3d(0, 0, Math.PI));
    }
  }

  @Override
  public void updateInputs(CoralOuttakeIOInputs inputs) {
    update();

    inputs.hasCoral = hasCoralSim;
    if (hasCoralSim) {
      // Visualize the coral in the rollers
      Logger.recordOutput(
          "Sim/Robot Coral",
          new Pose3d(Robot.groundTruthPoseSupplier.get())
              .plus(
                  new Transform3d(
                      Units.inchesToMeters(9),
                      0,
                      elevatorHeight.getAsDouble() + .5,
                      new Rotation3d(0, Units.degreesToRadians(30), 0))));
    } else {
      Logger.recordOutput("Sim/Robot Coral", new Pose3d(0, 0, -100, Rotation3d.kZero));
    }

    // Go through all the coral scored flags and add the corresponding pose to the list if
    // there's a score.
    var coralList = new ArrayList<Pose3d>();
    for (int i = 0; i < 12; i++) {
      if (l2OnBlue[i]) {
        coralList.add(l2BluePoses[i]);
      }
      if (l3OnBlue[i]) {
        coralList.add(l3BluePoses[i]);
      }
      if (l4OnBlue[i]) {
        coralList.add(l4BluePoses[i]);
      }
      if (l2OnRed[i]) {
        coralList.add(l2RedPoses[i]);
      }
      if (l3OnRed[i]) {
        coralList.add(l3RedPoses[i]);
      }
      if (l4OnRed[i]) {
        coralList.add(l4RedPoses[i]);
      }
    }
    // Visualize the list
    Logger.recordOutput("Sim/Reef coral", coralList.toArray(new Pose3d[0]));
  }

  @Override
  public void intake() {
    state = SimState.INTAKING;
  }

  @Override
  public void outtakeFast() {
    state = SimState.SCORING_SLOW;
  }

  @Override
  public void outtakeSlow() {
    state = SimState.SCORING_SLOW;
  }

  @Override
  public void agitate() {
    state = SimState.AGITATING;
  }

  @Override
  public void brake() {
    state = SimState.BRAKE;
  }

  private boolean hasCoralSim = true;

  private enum SimState {
    INTAKING,
    SCORING_FAST,
    SCORING_SLOW,
    AGITATING,
    BRAKE
  }

  private SimState state = SimState.BRAKE;

  private final Debouncer debouncer = new Debouncer(.5);

  double timer = 0;

  private void update() {
    Logger.recordOutput("Superstructure/Coral Outtake (Sim)/State", state.name());
    switch (state) {
      case INTAKING -> {
        timer = 0;
        if (!hasCoralSim) {
          hasCoralSim = debouncer.calculate(nearSource());
        }
      }
      case SCORING_SLOW -> {
        if (timer >= .15) {
          hasCoralSim = false;
          addCoralToReef();
        } else {
          timer += Robot.defaultPeriodSecs;
        }
      }
      case SCORING_FAST, AGITATING -> {
        if (timer >= .1) {
          hasCoralSim = false;
          addCoralToReef();
        } else {
          timer += Robot.defaultPeriodSecs;
        }
      }
      case BRAKE -> timer = 0;
    }
  }

  private final boolean[] l2OnBlue = new boolean[12];
  private final boolean[] l3OnBlue = new boolean[12];
  private final boolean[] l4OnBlue = new boolean[12];
  private final boolean[] l2OnRed = new boolean[12];
  private final boolean[] l3OnRed = new boolean[12];
  private final boolean[] l4OnRed = new boolean[12];

  private void addCoralToReef() {
    var robotPose = Robot.groundTruthPoseSupplier.get();
    var height = elevatorHeight.getAsDouble();

    // Select which set of branches to choose from
    // Blue or red, l2-l4
    boolean[] branches;
    Pose2d[] branchPoses;
    if (robotPose.getX() < Constants.FIELD_LENGTH_METERS / 2) {
      branchPoses = ReefLocations.BLUE_BRANCH_POSES;
      if (height < .7) {
        branches = l2OnBlue;
      } else if (height < 1.2) {
        branches = l3OnBlue;
      } else {
        branches = l4OnBlue;
      }
    } else {
      branchPoses = ReefLocations.RED_BRANCH_POSES;
      if (height < .7) {
        branches = l2OnRed;
      } else if (height < 1.2) {
        branches = l3OnRed;
      } else {
        branches = l4OnRed;
      }
    }

    // Search through all the scoring locations and pick the closest one
    int closestBranchIndex = 0;
    double closestBranchDistance = Double.POSITIVE_INFINITY;
    for (int i = 0; i < 12; i++) {
      var dist = branchPoses[i].minus(robotPose).getTranslation().getNorm();
      if (dist < closestBranchDistance) {
        closestBranchIndex = i;
        closestBranchDistance = dist;
      }
    }

    // If the robot is close enough, flag it to be scored.
    if (closestBranchDistance < .1) {
      branches[closestBranchIndex] = true;
    }
  }

  private final Rectangle2d rightSourceBoundingBox =
      new Rectangle2d(
          new Pose2d(1.75 / 2, 1.25 / 2, Rotation2d.fromDegrees(-54)),
          Math.hypot(1.75, 1.25),
          Constants.BUMPER_THICKNESS_METERS * 2 + Constants.FRAME_LENGTH_METERS + .5);
  private final Rectangle2d leftSourceBoundingBox =
      new Rectangle2d(
          new Pose2d(
              1.75 / 2, Constants.FIELD_WIDTH_METERS - (1.25 / 2), Rotation2d.fromDegrees(54)),
          Math.hypot(1.75, 1.25),
          Constants.BUMPER_THICKNESS_METERS * 2 + Constants.FRAME_LENGTH_METERS + .5);

  private boolean nearSource() {
    if (Robot.groundTruthPoseSupplier != null) {
      var robotPose = Robot.groundTruthPoseSupplier.get();

      if (Robot.isOnRed()) {
        robotPose = robotPose.rotateAround(Constants.FIELD_CENTER, Rotation2d.kPi);
      }

      boolean inRightBoundingBox = rightSourceBoundingBox.contains(robotPose.getTranslation());
      boolean inLeftBoundingBox = leftSourceBoundingBox.contains(robotPose.getTranslation());
      boolean facingRightSource = MathUtil.isNear(54, robotPose.getRotation().getDegrees(), 10);
      boolean facingLeftSource = MathUtil.isNear(-54, robotPose.getRotation().getDegrees(), 10);

      Logger.recordOutput(
          "Superstructure/Coral Outtake (Sim)/In right bounding box", inRightBoundingBox);
      Logger.recordOutput(
          "Superstructure/Coral Outtake (Sim)/In left bounding box", inLeftBoundingBox);
      Logger.recordOutput(
          "Superstructure/Coral Outtake (Sim)/Facing right source", facingRightSource);
      Logger.recordOutput(
          "Superstructure/Coral Outtake (Sim)/Facing left source", facingLeftSource);

      return (inLeftBoundingBox && facingLeftSource) || (inRightBoundingBox && facingRightSource);
    }
    return false;
  }
}
