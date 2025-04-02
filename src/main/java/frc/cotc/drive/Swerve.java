// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.drive;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static frc.cotc.drive.SwerveSetpointGenerator.SwerveSetpoint;
import static java.lang.Math.PI;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.cotc.Constants;
import frc.cotc.Robot;
import frc.cotc.util.ReefLocations;
import frc.cotc.vision.FiducialPoseEstimator;
import frc.cotc.vision.FiducialPoseEstimatorIOPhoton;
import java.util.Arrays;
import java.util.Comparator;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private final SwerveIO swerveIO;

  private final SwerveIO.SwerveIOInputs inputs;

  private final SwerveSetpointGenerator setpointGenerator;
  private final Rotation2d[] stopAngles;
  private final SwerveSetpoint stopInXSetpoint;
  private SwerveSetpoint lastSetpoint;

  private final double maxLinearSpeedMetersPerSec;
  private final double maxAngularSpeedRadPerSec;
  private final double angularSpeedFudgeFactor;

  private final SwervePoseEstimator poseEstimator;
  private final TimeInterpolatableBuffer<Rotation2d> yawBuffer =
      TimeInterpolatableBuffer.createBuffer(.25);
  private final FiducialPoseEstimator[] fiducialPoseEstimators;

  private final PIDController xController, yController, yawController;

  private final RepulsorFieldPlanner repulsorFieldPlanner = new RepulsorFieldPlanner();

  private final DoubleSupplier elevatorExtensionSupplier;

  public Swerve(
      SwerveIO driveIO,
      FiducialPoseEstimator.IO[] visionIOs,
      DoubleSupplier elevatorExtensionSupplier) {
    swerveIO = driveIO;
    var CONSTANTS = driveIO.getConstants();
    inputs = new SwerveIO.SwerveIOInputs();
    driveIO.updateInputs(inputs);
    Logger.processInputs("Swerve", inputs);
    Logger.processInputs("Swerve/Constants", CONSTANTS);

    maxLinearSpeedMetersPerSec =
        CONSTANTS.DRIVE_MOTOR.freeSpeedRadPerSec * (CONSTANTS.WHEEL_DIAMETER_METERS / 2);
    maxAngularSpeedRadPerSec =
        maxLinearSpeedMetersPerSec
            / Math.hypot(CONSTANTS.TRACK_WIDTH_METERS / 2, CONSTANTS.TRACK_LENGTH_METERS / 2);
    angularSpeedFudgeFactor = CONSTANTS.ANGULAR_SPEED_FUDGING;

    Logger.recordOutput("Swerve/Max Linear Speed", maxLinearSpeedMetersPerSec);
    Logger.recordOutput("Swerve/Max Angular Speed", maxAngularSpeedRadPerSec);

    setpointGenerator =
        new SwerveSetpointGenerator(
            new Translation2d[] {
              new Translation2d(
                  CONSTANTS.TRACK_LENGTH_METERS / 2, CONSTANTS.TRACK_WIDTH_METERS / 2),
              new Translation2d(
                  CONSTANTS.TRACK_LENGTH_METERS / 2, -CONSTANTS.TRACK_WIDTH_METERS / 2),
              new Translation2d(
                  -CONSTANTS.TRACK_LENGTH_METERS / 2, CONSTANTS.TRACK_WIDTH_METERS / 2),
              new Translation2d(
                  -CONSTANTS.TRACK_LENGTH_METERS / 2, -CONSTANTS.TRACK_WIDTH_METERS / 2)
            },
            CONSTANTS.DRIVE_MOTOR,
            CONSTANTS.SLIP_CURRENT_AMPS,
            CONSTANTS.MAX_STEER_SPEEDS_RAD_PER_SEC,
            CONSTANTS.MASS_KG,
            CONSTANTS.MOI_KG_METERS_SQUARED,
            CONSTANTS.WHEEL_DIAMETER_METERS);
    stopAngles =
        new Rotation2d[] {Rotation2d.kZero, Rotation2d.kZero, Rotation2d.kZero, Rotation2d.kZero};
    stopInXSetpoint =
        new SwerveSetpoint(
            new ChassisSpeeds(),
            new SwerveModuleState[] {
              new SwerveModuleState(0, stopAngles[0]),
              new SwerveModuleState(0, stopAngles[1]),
              new SwerveModuleState(0, stopAngles[2]),
              new SwerveModuleState(0, stopAngles[3])
            },
            new double[4],
            new double[4]);
    lastSetpoint =
        new SwerveSetpoint(new ChassisSpeeds(), inputs.moduleStates, new double[4], new double[4]);

    poseEstimator =
        new SwervePoseEstimator(
            setpointGenerator.getKinematics(),
            inputs.gyroYaw,
            getLatestModulePositions(),
            new Pose2d());

    fiducialPoseEstimators = new FiducialPoseEstimator[visionIOs.length];
    for (int i = 0; i < visionIOs.length; i++) {
      fiducialPoseEstimators[i] =
          new FiducialPoseEstimator(
              visionIOs[i].io(),
              visionIOs[i].name(),
              timestamp ->
                  DriverStation.isEnabled() ? yawBuffer.getSample(timestamp).orElse(null) : null,
              poseEstimator::getEstimatedPosition);
    }

    xController = new PIDController(5, 0, 0);
    yController = new PIDController(5, 0, 0);
    yawController = new PIDController(3, 0, .2);
    yawController.enableContinuousInput(-PI, PI);

    this.elevatorExtensionSupplier = elevatorExtensionSupplier;
  }

  private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
  private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();

  private final Alert invalidOdometryWarning =
      new Alert("Swerve: Invalid odometry data!", Alert.AlertType.kWarning);
  private final Alert outOfOrderOdometryWarning =
      new Alert(
          "Swerve: Odometry data was out of order! Expected latest data last.",
          Alert.AlertType.kWarning);

  @Override
  public void periodic() {
    swerveIO.updateInputs(inputs);
    Logger.processInputs("Swerve", inputs);
    robotRelativeSpeeds = getRobotChassisSpeeds();
    Logger.recordOutput("Swerve/Actual Speed", robotRelativeSpeeds);

    fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, inputs.gyroYaw);

    if (Robot.isSimulation() && !Logger.hasReplaySource()) {
      FiducialPoseEstimatorIOPhoton.Sim.update();
    }

    if (!poseReset) {
      var driveStdDevs = getDriveStdDevs();
      Logger.recordOutput("Swerve/Odometry/Drive Std Devs", driveStdDevs);
      poseEstimator.setDriveMeasurementStdDevs(driveStdDevs);

      double lastTimestamp = -1;
      for (var frame : inputs.odometryFrames) {
        if (frame.timestamp() < 0) {
          invalidOdometryWarning.set(true);
          break;
        }
        invalidOdometryWarning.set(false);
        if (frame.timestamp() < lastTimestamp) {
          outOfOrderOdometryWarning.set(true);
          break;
        }
        outOfOrderOdometryWarning.set(false);
        poseEstimator.updateWithTime(frame.timestamp(), frame.gyroYaw(), frame.positions());
        yawBuffer.addSample(frame.timestamp(), frame.gyroYaw());
        lastTimestamp = frame.timestamp();
      }

      var poseEstimates = new FiducialPoseEstimator.PoseEstimate[0];
      for (var fiducialPoseEstimator : fiducialPoseEstimators) {
        var polledEstimates = fiducialPoseEstimator.poll();

        var newSignals =
            new FiducialPoseEstimator.PoseEstimate[poseEstimates.length + polledEstimates.length];
        System.arraycopy(poseEstimates, 0, newSignals, 0, poseEstimates.length);
        System.arraycopy(
            polledEstimates, 0, newSignals, poseEstimates.length, polledEstimates.length);
        poseEstimates = newSignals;
      }
      Arrays.sort(
          poseEstimates, Comparator.comparingDouble(FiducialPoseEstimator.PoseEstimate::timestamp));
      for (var poseEstimate : poseEstimates) {
        poseEstimator.addVisionMeasurement(
            poseEstimate.pose(),
            poseEstimate.timestamp(),
            new double[] {
              poseEstimate.translationalStdDevs(),
              poseEstimate.translationalStdDevs(),
              poseEstimate.yawStdDevs()
            });
      }
      var robotPose3d = new Pose3d(poseEstimator.getEstimatedPosition());
      for (var fiducialPoseEstimator : fiducialPoseEstimators) {
        Logger.recordOutput(
            "Vision/" + fiducialPoseEstimator.name + "/cameraPose",
            robotPose3d.plus(fiducialPoseEstimator.robotToCameraTransform));
      }
    } else {
      poseReset = false;
    }

    Logger.recordOutput("Swerve/Odometry/Final Position", poseEstimator.getEstimatedPosition());
  }

  /**
   * Estimate drive wheel slippage by comparing the actual wheel velocities to the idealized wheel
   * velocities. If there is a significant deviation, then a wheel(s) is slipping, and we should
   * raise the estimated standard deviation of the drivebase odometry to trust the wheel encoders
   * less.
   *
   * @return An array of length 3, containing the estimated standard deviations in each axis (x, y,
   *     yaw)
   */
  private double[] getDriveStdDevs() {
    // Get idealized states from the current robot velocity.
    var idealStates = setpointGenerator.getKinematics().toSwerveModuleStates(robotRelativeSpeeds);

    double xSquaredSum = 0;
    double ySquaredSum = 0;
    for (int i = 0; i < 4; i++) {
      var measuredVector =
          new Translation2d(
              inputs.moduleStates[i].speedMetersPerSecond, inputs.moduleStates[i].angle);
      var idealVector =
          new Translation2d(idealStates[i].speedMetersPerSecond, idealStates[i].angle);

      // Compare the state vectors and get the delta between them.
      var xDelta = idealVector.getX() - measuredVector.getX();
      var yDelta = idealVector.getY() - measuredVector.getY();

      // Square the delta and add it to a sum
      xSquaredSum += xDelta * xDelta;
      ySquaredSum += yDelta * yDelta;
    }

    // Sqrt of avg of squared deltas = standard deviation
    // Rotate to convert to field relative
    double scalar = 15;
    var stdDevs =
        new Translation2d(
                scalar * (Math.sqrt(xSquaredSum) / 4), scalar * (Math.sqrt(ySquaredSum) / 4))
            .rotateBy(inputs.gyroYaw);

    // If translating and rotating at the same time, odometry drifts pretty badly in the
    // direction perpendicular to the direction of translational travel.
    // This factor massively distrusts odometry in that direction when translating and rotating
    // at the same time.
    var scaledSpeed =
        new Translation2d(
                fieldRelativeSpeeds.vxMetersPerSecond / maxLinearSpeedMetersPerSec,
                fieldRelativeSpeeds.vyMetersPerSecond / maxLinearSpeedMetersPerSec)
            .rotateBy(Rotation2d.kCCW_90deg)
            .times(
                1 * Math.abs(fieldRelativeSpeeds.omegaRadiansPerSecond / maxAngularSpeedRadPerSec));

    // Add a minimum to account for mechanical slop and to prevent divide by 0 errors
    return new double[] {
      Math.abs(stdDevs.getX()) + Math.abs(scaledSpeed.getX()) + .1,
      Math.abs(stdDevs.getY()) + Math.abs(scaledSpeed.getY()) + .1,
      .001
    };
  }

  private void drive(ChassisSpeeds desiredSpeeds) {
    double elevatorHeight = elevatorExtensionSupplier.getAsDouble();

    double maxForwardAccel = MathUtil.interpolate(14, 12, elevatorHeight);
    double maxReverseAccel =
        MathUtil.interpolate(
            14, lastSetpoint.chassisSpeeds().vxMetersPerSecond > 0 ? 12 : 4, elevatorHeight);
    double maxSideAccel = MathUtil.interpolate(14, 12, elevatorHeight);

    Logger.recordOutput("Swerve/Accel Limits/Forward", maxForwardAccel);
    Logger.recordOutput("Swerve/Accel Limits/Reverse", -maxReverseAccel);
    Logger.recordOutput("Swerve/Accel Limits/Side", maxSideAccel);

    var desiredXAccel =
        (desiredSpeeds.vxMetersPerSecond - lastSetpoint.chassisSpeeds().vxMetersPerSecond)
            / Robot.defaultPeriodSecs;
    var desiredYAccel =
        (desiredSpeeds.vyMetersPerSecond - lastSetpoint.chassisSpeeds().vyMetersPerSecond)
            / Robot.defaultPeriodSecs;

    double scalar = maxSideAccel / Math.max(Math.abs(desiredYAccel), maxSideAccel);
    scalar =
        Math.min(
            scalar,
            desiredXAccel > 0
                ? maxForwardAccel / Math.max(desiredXAccel, maxForwardAccel)
                : maxReverseAccel / Math.max(-desiredXAccel, maxReverseAccel));

    ChassisSpeeds limitedSpeeds;
    if (scalar == 1) {
      limitedSpeeds = desiredSpeeds;
    } else {
      limitedSpeeds =
          new ChassisSpeeds(
              lastSetpoint.chassisSpeeds().vxMetersPerSecond
                  + desiredXAccel * Robot.defaultPeriodSecs * scalar,
              lastSetpoint.chassisSpeeds().vyMetersPerSecond
                  + desiredYAccel * Robot.defaultPeriodSecs * scalar,
              lastSetpoint.chassisSpeeds().omegaRadiansPerSecond
                  + (desiredSpeeds.omegaRadiansPerSecond
                          - lastSetpoint.chassisSpeeds().omegaRadiansPerSecond)
                      * scalar);
    }
    var setpoint = setpointGenerator.generateSetpoint(lastSetpoint, limitedSpeeds);
    swerveIO.drive(setpoint);
    lastSetpoint = setpoint;
  }

  public Command teleopDrive(
      Supplier<Translation2d> translationalControlSupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier slowMode) {
    return run(() -> {
          var translationalControl = translationalControlSupplier.get();

          var commandedRobotSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  new ChassisSpeeds(
                      translationalControl.getX() * maxLinearSpeedMetersPerSec,
                      translationalControl.getY() * maxLinearSpeedMetersPerSec,
                      omegaSupplier.getAsDouble() * maxAngularSpeedRadPerSec),
                  inputs.gyroYaw);

          commandedRobotSpeeds.omegaRadiansPerSecond *=
              1 - translationalControl.getNorm() * angularSpeedFudgeFactor;

          if (slowMode.getAsBoolean()) {
            commandedRobotSpeeds = commandedRobotSpeeds.times(.3);
          }

          drive(commandedRobotSpeeds);
        })
        .withName("Teleop Drive");
  }

  public Command stop() {
    return run(() -> {
          swerveIO.stop(stopAngles);
          lastSetpoint = stopInXSetpoint;
        })
        .ignoringDisable(true)
        .withName("Stop");
  }

  public Command resetGyro() {
    return runOnce(
            () -> {
              var gyroAngle =
                  Robot.isOnRed()
                      ? poseEstimator.getEstimatedPosition().getRotation().rotateBy(Rotation2d.kPi)
                      : poseEstimator.getEstimatedPosition().getRotation();
              swerveIO.resetGyro(gyroAngle);
              poseEstimator.resetPosition(
                  gyroAngle, getLatestModulePositions(), poseEstimator.getEstimatedPosition());
            })
        .withName("Reset Gyro");
  }

  public void followChoreoTrajectory(SwerveSample sample) {
    Logger.recordOutput(
        "Choreo/Target pose", new Pose2d(sample.x, sample.y, new Rotation2d(sample.heading)));

    var feedforward = new ChassisSpeeds(sample.vx, sample.vy, sample.omega);
    Logger.recordOutput("Choreo/Feedforward (Field)", feedforward);
    var feedback =
        new ChassisSpeeds(
            xController.calculate(poseEstimator.getEstimatedPosition().getX(), sample.x),
            yController.calculate(poseEstimator.getEstimatedPosition().getY(), sample.y),
            yawController.calculate(
                poseEstimator.getEstimatedPosition().getRotation().getRadians(), sample.heading));
    Logger.recordOutput("Choreo/Feedback (Field)", feedback);

    var outputFieldRelative = feedforward.plus(feedback);
    var outputRobotRelative =
        ChassisSpeeds.fromFieldRelativeSpeeds(outputFieldRelative, new Rotation2d(sample.heading));

    drive(outputRobotRelative);
  }

  private Pose2d targetPose;

  @AutoLogOutput
  public boolean atTargetPoseTeleop() {
    if (targetPose == null) {
      return false;
    }
    var error = targetPose.minus(poseEstimator.getEstimatedPosition());
    Logger.recordOutput("Swerve/Error", error.getTranslation().getNorm());
    return error.getTranslation().getNorm() < .05
        && Math.abs(error.getRotation().getDegrees()) < 5
        && Math.hypot(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond)
            < .5
        && Math.abs(Units.radiansToDegrees(fieldRelativeSpeeds.omegaRadiansPerSecond)) < 10;
  }

  @AutoLogOutput
  public boolean atTargetPoseAuto() {
    if (targetPose == null) {
      return false;
    }
    var error = targetPose.minus(poseEstimator.getEstimatedPosition());
    return error.getTranslation().getNorm() < .08
        && Math.abs(error.getRotation().getDegrees()) < 5
        && Math.hypot(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond)
            < .5
        && Math.abs(Units.radiansToDegrees(fieldRelativeSpeeds.omegaRadiansPerSecond)) < 10;
  }

  @AutoLogOutput
  public boolean nearingTargetPose() {
    if (targetPose == null) {
      return false;
    }
    var error = targetPose.minus(poseEstimator.getEstimatedPosition());
    return error.getTranslation().getNorm() < 1.5
        && Math.abs(error.getRotation().getDegrees()) < 20;
  }

  public Command netAlign(Supplier<Translation2d> translationalControlSupplier) {
    return runOnce(
            () -> {
              xController.reset();
              yawController.reset();
            })
        .andThen(
            run(
                () -> {
                  var targetX = 7.825;
                  if (Robot.isOnRed()) {
                    targetX = Constants.FIELD_LENGTH_METERS - targetX;
                  }

                  var xOutput =
                      xController.calculate(poseEstimator.getEstimatedPosition().getX(), targetX);
                  var yawOutput =
                      yawController.calculate(
                          poseEstimator.getEstimatedPosition().getRotation().getRadians(),
                          Robot.isOnRed() ? 0 : PI);

                  var driverInput =
                      Robot.isOnRed()
                          ? translationalControlSupplier.get().unaryMinus()
                          : translationalControlSupplier.get();
                  var outputFieldRelative =
                      new ChassisSpeeds(
                          xOutput + driverInput.getX() * maxLinearSpeedMetersPerSec * .5,
                          driverInput.getY() * maxLinearSpeedMetersPerSec * .5,
                          yawOutput);
                  Logger.recordOutput("Swerve/Net align/Output", outputFieldRelative);

                  drive(
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          outputFieldRelative, poseEstimator.getEstimatedPosition().getRotation()));
                }))
        .withName("Net Align");
  }

  public Command followRepulsorField(Pose2d goal) {
    return followRepulsorField(() -> goal, null);
  }

  private Command followRepulsorField(
      Supplier<Pose2d> goal, Supplier<Translation2d> nudgeSupplier) {
    return sequence(
            runOnce(
                () -> {
                  targetPose = goal.get();
                  repulsorFieldPlanner.setGoal(targetPose.getTranslation());
                  xController.reset();
                  yController.reset();
                  yawController.reset();
                }),
            run(
                () -> {
                  Logger.recordOutput("Repulsor/Goal", targetPose);

                  var sample =
                      repulsorFieldPlanner.sampleField(
                          poseEstimator.getEstimatedPosition().getTranslation(),
                          maxLinearSpeedMetersPerSec * .9,
                          .8);

                  var feedforward = new ChassisSpeeds(sample.vx(), sample.vy(), 0);
                  var feedback =
                      new ChassisSpeeds(
                          xController.calculate(
                              poseEstimator.getEstimatedPosition().getX(),
                              sample.intermediateGoal().getX()),
                          yController.calculate(
                              poseEstimator.getEstimatedPosition().getY(),
                              sample.intermediateGoal().getY()),
                          yawController.calculate(
                              poseEstimator.getEstimatedPosition().getRotation().getRadians(),
                              targetPose.getRotation().getRadians()));

                  var error = targetPose.minus(poseEstimator.getEstimatedPosition());
                  Logger.recordOutput("Repulsor/Error", error);
                  Logger.recordOutput("Repulsor/Feedforward", feedforward);
                  Logger.recordOutput("Repulsor/Feedback", feedback);

                  //                  Logger.recordOutput("Repulsor/Vector field",
                  // repulsorFieldPlanner.getArrows());

                  var outputFieldRelative = feedforward.plus(feedback);

                  if (nudgeSupplier != null) {
                    var nudge = nudgeSupplier.get();
                    if (nudge.getNorm() > .1) {
                      var nudgeScalar =
                          Math.min(error.getTranslation().getNorm() / 3, 1)
                              * Math.min(error.getTranslation().getNorm() / 3, 1)
                              * maxLinearSpeedMetersPerSec;

                      if (Robot.isOnRed()) {
                        nudge = new Translation2d(-nudge.getX(), -nudge.getY());
                      }
                      nudgeScalar *=
                          Math.abs(
                              nudge
                                  .getAngle()
                                  .minus(
                                      new Rotation2d(
                                          outputFieldRelative.vxMetersPerSecond,
                                          outputFieldRelative.vyMetersPerSecond))
                                  .getSin());
                      outputFieldRelative.vxMetersPerSecond += nudge.getX() * nudgeScalar;
                      outputFieldRelative.vyMetersPerSecond += nudge.getY() * nudgeScalar;
                    }
                  }

                  var outputRobotRelative =
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          outputFieldRelative, poseEstimator.getEstimatedPosition().getRotation());

                  drive(outputRobotRelative);
                }))
        .withName("Repulsor Field");
  }

  public Command reefBranchAlign(boolean left, Supplier<Translation2d> nudgeSupplier) {
    var bluePoses = new Pose2d[6];
    var redPoses = new Pose2d[6];
    for (int i = 0; i < 6; i++) {
      bluePoses[i] = ReefLocations.BLUE_BRANCH_POSES[i * 2 + (left ? 0 : 1)];
      redPoses[i] = ReefLocations.RED_BRANCH_POSES[i * 2 + (left ? 0 : 1)];
    }
    return followRepulsorField(
            () ->
                Robot.isOnRed()
                    ? selectPose(redPoses, nudgeSupplier.get().unaryMinus())
                    : selectPose(bluePoses, nudgeSupplier.get()),
            nudgeSupplier)
        .withName("Reef branch align " + (left ? "left" : "right"));
  }

  private Pose2d reefAlignPose;

  public Command reefAlgaeAlign(Supplier<Translation2d> nudgeSupplier) {
    return followRepulsorField(
            () -> {
              reefAlignPose =
                  Robot.isOnRed()
                      ? selectPose(ReefLocations.RED_ALGAE_POSES, nudgeSupplier.get().unaryMinus())
                      : selectPose(ReefLocations.BLUE_ALGAE_POSES, nudgeSupplier.get());
              return reefAlignPose;
            },
            nudgeSupplier)
        .finallyDo(() -> reefAlignPose = null)
        .withName("Reef algae align");
  }

  public Pose2d getReefAlignPose() {
    if (reefAlignPose == null) {
      return selectPose(
          Robot.isOnRed() ? ReefLocations.RED_ALGAE_POSES : ReefLocations.BLUE_ALGAE_POSES,
          Translation2d.kZero);
    } else {
      return reefAlignPose;
    }
  }

  private Pose2d selectPose(Pose2d[] poses, Translation2d driverControlVector) {
    int bestPose = 0;
    double bestScore = Double.POSITIVE_INFINITY;
    for (int i = 0; i < poses.length; i++) {
      var location = poses[i].getTranslation();

      var robotToPoseVector = location.minus(poseEstimator.getEstimatedPosition().getTranslation());

      var distanceScore = robotToPoseVector.getNorm();

      double driverInputScore;
      if (driverControlVector.getNorm() < .1) {
        driverInputScore = 0;
      } else {
        var robotToBranchAngle = robotToPoseVector.getAngle();
        var driverControlAngle = driverControlVector.getAngle();

        driverInputScore = driverControlAngle.minus(robotToBranchAngle).getCos() * 2;
      }

      Logger.recordOutput("Swerve/Smart Align/Branch " + i + "/Distance score", distanceScore);
      Logger.recordOutput(
          "Swerve/Smart Align/Branch " + i + "/Driver input score", driverInputScore);
      double branchScore = distanceScore - driverInputScore;
      Logger.recordOutput("Swerve/Smart Align/Branch " + i + "/Overall score", branchScore);

      if (branchScore < bestScore) {
        bestPose = i;
        bestScore = branchScore;
      }
    }

    return poses[bestPose];
  }

  public Command testSlipCurrent() {
    var timer = new Timer();

    return runOnce(timer::restart)
        .andThen(
            run(() -> swerveIO.testSlipCurrent(Math.max(timer.get() - 1, 0) * 5))
                .until(
                    () -> {
                      for (int i = 0; i < 4; i++) {
                        if (Math.abs(inputs.moduleStates[i].speedMetersPerSecond) > 1) {
                          System.out.println(
                              "Current slip detected at "
                                  + (Math.max(timer.get() - 1, 0) * 5)
                                  + "amps");
                          return true;
                        }
                      }
                      return false;
                    }))
        .finallyDo(
            () ->
                lastSetpoint =
                    new SwerveSetpoint(
                        getRobotChassisSpeeds(),
                        inputs.moduleStates,
                        new double[4],
                        new double[4]));
  }

  public Command lockForward() {
    var angles = new Rotation2d[4];
    Arrays.fill(angles, Rotation2d.kZero);

    return run(() -> swerveIO.stop(angles));
  }

  public Command lockBackwards() {
    var angles = new Rotation2d[4];
    Arrays.fill(angles, Rotation2d.kPi);

    return run(() -> swerveIO.stop(angles));
  }

  private ChassisSpeeds getRobotChassisSpeeds() {
    return setpointGenerator.getKinematics().toChassisSpeeds(inputs.moduleStates);
  }

  public SwerveModulePosition[] getLatestModulePositions() {
    if (inputs.odometryFrames.length == 0) {
      throw new IndexOutOfBoundsException(
          "swerveInputs.odometryFrames.length was 0! This should not be possible.");
    }
    return inputs.odometryFrames[inputs.odometryFrames.length - 1].positions();
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  private boolean poseReset = false;

  public void resetForAuto(Pose2d pose) {
    DriverStation.reportWarning("Reset For Auto run", true);
    if (Robot.isSimulation() && !Logger.hasReplaySource()) {
      if (swerveIO instanceof SwerveIOPhoenix phoenix) {
        phoenix.resetGroundTruth(pose);
      }
    }
    var gyroAngle =
        Robot.isOnRed() ? pose.getRotation().rotateBy(Rotation2d.kPi) : pose.getRotation();
    swerveIO.resetGyro(gyroAngle);
    xController.reset();
    yController.reset();
    yawController.reset();
    poseEstimator.resetPosition(gyroAngle, getLatestModulePositions(), pose);
    swerveIO.updateInputs(inputs);
    Logger.processInputs("Swerve", inputs);
    poseReset = true;
  }
}
