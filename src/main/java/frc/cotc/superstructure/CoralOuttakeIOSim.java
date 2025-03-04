// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.cotc.Constants;
import frc.cotc.Robot;
import org.littletonrobotics.junction.Logger;

public class CoralOuttakeIOSim implements CoralOuttakeIO {
  @Override
  public void updateInputs(CoralOuttakeIOInputs inputs) {
    update();

    inputs.hasCoral = hasCoralSim;
  }

  @Override
  public void intake() {
    state = SimState.INTAKING;
  }

  @Override
  public void outtake() {
    state = SimState.SCORING;
  }

  @Override
  public void agitate() {
    state = SimState.AGITATING;
  }

  @Override
  public void brake() {
    state = SimState.BRAKE;
  }

  private boolean hasCoralSim = false;

  private enum SimState {
    INTAKING,
    SCORING,
    AGITATING,
    BRAKE
  }

  private SimState state = SimState.BRAKE;

  private void update() {
    Logger.recordOutput("Superstructure/Coral Outtake (Sim)/State", state.name());
    switch (state) {
      case INTAKING -> {
        if (!hasCoralSim) {
          hasCoralSim = nearSource();
        }
      }
      case SCORING, AGITATING -> hasCoralSim = false;
      case BRAKE -> {}
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
