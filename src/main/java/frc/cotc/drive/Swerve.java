// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.drive;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.cotc.drive.SwerveSetpointGenerator.SwerveSetpoint;
import static java.lang.Math.PI;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
  private final Rotation2d[] stopInXAngles;
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

  public Swerve(SwerveIO driveIO, FiducialPoseEstimator.IO[] visionIOs) {
    this.swerveIO = driveIO;
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
            CONSTANTS.DRIVE_STATOR_CURRENT_LIMIT_AMPS,
            CONSTANTS.MAX_STEER_SPEEDS_RAD_PER_SEC,
            CONSTANTS.MASS_KG,
            CONSTANTS.MOI_KG_METERS_SQUARED,
            CONSTANTS.WHEEL_DIAMETER_METERS,
            CONSTANTS.WHEEL_COF);
    stopInXAngles =
        new Rotation2d[] {
          new Rotation2d(CONSTANTS.TRACK_WIDTH_METERS / 2, CONSTANTS.TRACK_LENGTH_METERS / 2),
          new Rotation2d(CONSTANTS.TRACK_WIDTH_METERS / 2, -CONSTANTS.TRACK_LENGTH_METERS / 2),
          new Rotation2d(-CONSTANTS.TRACK_WIDTH_METERS / 2, CONSTANTS.TRACK_LENGTH_METERS / 2),
          new Rotation2d(-CONSTANTS.TRACK_WIDTH_METERS / 2, -CONSTANTS.TRACK_LENGTH_METERS / 2)
        };
    stopInXSetpoint =
        new SwerveSetpoint(
            new ChassisSpeeds(),
            new SwerveModuleState[] {
              new SwerveModuleState(0, stopInXAngles[0]),
              new SwerveModuleState(0, stopInXAngles[1]),
              new SwerveModuleState(0, stopInXAngles[2]),
              new SwerveModuleState(0, stopInXAngles[3])
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

    xController = new PIDController(7.5, 0, 0);
    yController = new PIDController(7.5, 0, 0);
    yawController = new PIDController(4, 0, .2);
    yawController.enableContinuousInput(-PI, PI);
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

  private double accelLimitMpss = -1;

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

          if (accelLimitMpss > 0) {
            double desiredAccelXMpss =
                (commandedRobotSpeeds.vxMetersPerSecond
                        - lastSetpoint.chassisSpeeds().vxMetersPerSecond)
                    / Robot.defaultPeriodSecs;
            double desiredAccelYMpss =
                (commandedRobotSpeeds.vyMetersPerSecond
                        - lastSetpoint.chassisSpeeds().vyMetersPerSecond)
                    / Robot.defaultPeriodSecs;
            double desiredAccelMpss = Math.hypot(desiredAccelXMpss, desiredAccelYMpss);
            if (desiredAccelMpss > accelLimitMpss) {
              var accelDirRad = Math.atan2(desiredAccelYMpss, desiredAccelXMpss);
              commandedRobotSpeeds.vxMetersPerSecond =
                  lastSetpoint.chassisSpeeds().vxMetersPerSecond
                      + Math.cos(accelDirRad) * accelLimitMpss * Robot.defaultPeriodSecs;
              commandedRobotSpeeds.vyMetersPerSecond =
                  lastSetpoint.chassisSpeeds().vyMetersPerSecond
                      + Math.sin(accelDirRad) * accelLimitMpss * Robot.defaultPeriodSecs;
            }
          }

          commandedRobotSpeeds.omegaRadiansPerSecond *=
              1 - translationalControl.getNorm() * angularSpeedFudgeFactor;

          if (slowMode.getAsBoolean()) {
            commandedRobotSpeeds = commandedRobotSpeeds.times(.3);
          }

          var setpoint = setpointGenerator.generateSetpoint(lastSetpoint, commandedRobotSpeeds);
          swerveIO.drive(setpoint);
          lastSetpoint = setpoint;
        })
        .withName("Teleop Drive");
  }

  public Command stop() {
    return run(() -> {
          swerveIO.stop(stopInXAngles);
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

    var setpoint = setpointGenerator.generateSetpoint(lastSetpoint, outputRobotRelative);
    swerveIO.drive(setpoint);
    lastSetpoint = setpoint;
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
    return error.getTranslation().getNorm() < 1 && Math.abs(error.getRotation().getDegrees()) < 20;
  }

  public Command sourceAlign(Supplier<Translation2d> translationalControlSupplier) {
    return runOnce(yawController::reset)
        .andThen(
            run(
                () -> {
                  var targetAngle = Units.degreesToRadians(54);
                  if (Robot.isOnRed()) {
                    targetAngle = PI - targetAngle;
                  }
                  if (poseEstimator.getEstimatedPosition().getY()
                      > Constants.FIELD_WIDTH_METERS / 2) {
                    targetAngle *= -1;
                  }

                  var translationalControl = translationalControlSupplier.get();

                  var commandedRobotSpeeds =
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          new ChassisSpeeds(
                              translationalControl.getX() * maxLinearSpeedMetersPerSec,
                              translationalControl.getY() * maxLinearSpeedMetersPerSec,
                              yawController.calculate(
                                  poseEstimator.getEstimatedPosition().getRotation().getRadians(),
                                  targetAngle)),
                          inputs.gyroYaw);

                  var setpoint =
                      setpointGenerator.generateSetpoint(lastSetpoint, commandedRobotSpeeds);
                  swerveIO.drive(setpoint);
                  lastSetpoint = setpoint;
                }))
        .withName("SourceAlign");
  }

  public Command followRepulsorField(Pose2d goal) {
    return followRepulsorField(goal, null);
  }

  public Command followRepulsorField(Pose2d goal, Supplier<Translation2d> nudgeSupplier) {
    return sequence(
            runOnce(
                () -> {
                  repulsorFieldPlanner.setGoal(goal.getTranslation());
                  xController.reset();
                  yController.reset();
                  yawController.reset();
                  targetPose = goal;
                }),
            run(
                () -> {
                  Logger.recordOutput("Repulsor/Goal", goal);

                  var sample =
                      repulsorFieldPlanner.sampleField(
                          poseEstimator.getEstimatedPosition().getTranslation(),
                          maxLinearSpeedMetersPerSec * .75,
                          1.25);

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
                              goal.getRotation().getRadians()));

                  var error = goal.minus(poseEstimator.getEstimatedPosition());
                  Logger.recordOutput("Repulsor/Error", error);
                  Logger.recordOutput("Repulsor/Feedforward", feedforward);
                  Logger.recordOutput("Repulsor/Feedback", feedback);

                  //                  Logger.recordOutput("Repulsor/Vector field",
                  // repulsorFieldPlanner.getArrows());

                  var outputFieldRelative = feedforward.plus(feedback);

                  if (nudgeSupplier != null) {
                    var nudge = nudgeSupplier.get();

                    if (Robot.isOnRed()) {
                      nudge = nudge.unaryMinus();
                    }

                    var nudgeScalar = .5;
                    outputFieldRelative.vxMetersPerSecond += nudge.getX() * nudgeScalar;
                    outputFieldRelative.vyMetersPerSecond += nudge.getY() * nudgeScalar;
                  }

                  var outputRobotRelative =
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          outputFieldRelative, poseEstimator.getEstimatedPosition().getRotation());

                  var setpoint =
                      setpointGenerator.generateSetpoint(lastSetpoint, outputRobotRelative);
                  swerveIO.drive(setpoint);
                  lastSetpoint = setpoint;
                }))
        .withName("Repulsor Field");
  }

  public Command reefAlign(boolean left, Supplier<Translation2d> nudgeSupplier) {
    return defer(
            () -> {
              int bestBranch = 0;
              double bestScore = Double.POSITIVE_INFINITY;
              for (int i = 0; i < 6; i++) {
                var branchLocation = getBranchPose(i, left).getTranslation();

                var robotToBranchVector =
                    branchLocation.minus(poseEstimator.getEstimatedPosition().getTranslation());

                var branchDistanceScore = robotToBranchVector.getNorm();

                var driverControlVector = nudgeSupplier.get();
                if (Robot.isOnRed()) {
                  driverControlVector =
                      new Translation2d(-driverControlVector.getX(), -driverControlVector.getY());
                }

                double driverInputScore;
                if (driverControlVector.getNorm() < .1) {
                  driverInputScore = 0;
                } else {
                  var robotToBranchAngle = robotToBranchVector.getAngle();
                  var driverControlAngle = driverControlVector.getAngle();

                  driverInputScore = driverControlAngle.minus(robotToBranchAngle).getCos() * 2;
                }

                Logger.recordOutput(
                    "Swerve/Reef Align/Branch " + i + "/Distance score", branchDistanceScore);
                Logger.recordOutput(
                    "Swerve/Reef Align/Branch " + i + "/Driver input score", driverInputScore);
                double branchScore = branchDistanceScore - driverInputScore;
                Logger.recordOutput(
                    "Swerve/Reef Align/Branch " + i + "/Overall score", branchScore);

                if (branchScore < bestScore) {
                  bestBranch = i;
                  bestScore = branchScore;
                }
              }
              return followRepulsorField(getBranchPose(bestBranch, left), nudgeSupplier);
            })
        .withName("Reef align " + (left ? "left" : "right"));
  }

  private Pose2d getBranchPose(int reefWall, boolean left) {
    var branches = Robot.isOnRed() ? ReefLocations.RED_POSES : ReefLocations.BLUE_POSES;
    return branches[reefWall * 2 + (left ? 0 : 1)];
  }

  /**
   * Runs SysID for characterizing the drivebase.
   *
   * <p>Moment of inertia can be estimated using the following equation:
   *
   * <p>{@code I = mass * drivebaseRadius * kA_angular / kA_linear}
   *
   * <p>The above formula is from <a
   * href="https://choreo.autos/usage/estimating-moi/">https://choreo.autos/usage/estimating-moi/</a>
   *
   * @param linear Whether the routine should be characterizing linear or angular dynamics.
   */
  public Command driveCharacterize(boolean linear) {
    var states =
        setpointGenerator
            .getKinematics()
            .toSwerveModuleStates(linear ? new ChassisSpeeds(1, 0, 0) : new ChassisSpeeds(0, 0, 1));
    Rotation2d[] angles = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      angles[i] = states[i].angle;
    }

    var sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(10).per(Second),
                Volts.of(20),
                Seconds.of(4),
                state -> SignalLogger.writeString("SysIDState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> swerveIO.driveCharacterization(voltage.baseUnitMagnitude(), angles),
                null,
                this));

    return sequence(
        runOnce(swerveIO::initSysId),
        run(() -> swerveIO.driveCharacterization(0, angles)).withTimeout(1),
        sysId.quasistatic(SysIdRoutine.Direction.kForward),
        waitSeconds(1),
        sysId.quasistatic(SysIdRoutine.Direction.kReverse),
        waitSeconds(1),
        sysId.dynamic(SysIdRoutine.Direction.kForward),
        waitSeconds(1),
        sysId.dynamic(SysIdRoutine.Direction.kReverse));
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
