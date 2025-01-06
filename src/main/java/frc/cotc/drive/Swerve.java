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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.cotc.Robot;
import frc.cotc.vision.FiducialPoseEstimatorIO;
import frc.cotc.vision.FiducialPoseEstimatorIO.FiducialPoseEstimatorIOInputs;
import frc.cotc.vision.FiducialPoseEstimatorIOPhoton;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private final SwerveIO swerveIO;

  private final SwerveIO.SwerveIOInputs swerveInputs;

  private final SwerveSetpointGenerator setpointGenerator;
  private final SwerveSetpoint stopInXSetpoint;
  private SwerveSetpoint lastSetpoint;

  private final double maxLinearSpeedMetersPerSec;
  private final double maxAngularSpeedRadPerSec;
  private final double angularSpeedFudgeFactor;

  private final SwervePoseEstimator poseEstimator;

  private final FiducialPoseEstimatorIO[] visionIOs;
  private final FiducialPoseEstimatorIOInputs[] visionInputs;

  private final PIDController xController, yController, yawController;

  private record StdDevTunings(
      double distanceScalar, double heightScalar, double tagCountExponent) {}

  private record CameraTunings(StdDevTunings translational, StdDevTunings angular) {
    static final CameraTunings defaults =
        new CameraTunings(new StdDevTunings(1.5, 1, 2), new StdDevTunings(.2, 1, 1.5));
  }

  private final double wheelRadiusMeters;
  private final double kTNewtonMetersPerAmp;
  private final double currentVisualizationScalar;

  private final CameraTunings[] cameraTunings = new CameraTunings[0];

  public Swerve(SwerveIO driveIO, FiducialPoseEstimatorIO[] visionIOs) {
    this.swerveIO = driveIO;
    var CONSTANTS = driveIO.getConstants();
    swerveInputs = new SwerveIO.SwerveIOInputs();
    driveIO.updateInputs(swerveInputs);
    Logger.processInputs("Swerve", swerveInputs);
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
            CONSTANTS.MAX_STEER_SPEED_RAD_PER_SEC,
            CONSTANTS.MASS_KG,
            CONSTANTS.MOI_KG_METERS_SQUARED,
            CONSTANTS.WHEEL_DIAMETER_METERS,
            CONSTANTS.WHEEL_COF);
    stopInXSetpoint =
        new SwerveSetpoint(
            new ChassisSpeeds(),
            new SwerveModuleState[] {
              new SwerveModuleState(
                  0,
                  new Rotation2d(
                      Math.atan2(
                          CONSTANTS.TRACK_WIDTH_METERS / 2, CONSTANTS.TRACK_LENGTH_METERS / 2))),
              new SwerveModuleState(
                  0,
                  new Rotation2d(
                      Math.atan2(
                          -CONSTANTS.TRACK_WIDTH_METERS / 2, CONSTANTS.TRACK_LENGTH_METERS / 2))),
              new SwerveModuleState(
                  0,
                  new Rotation2d(
                      Math.atan2(
                          CONSTANTS.TRACK_WIDTH_METERS / 2, -CONSTANTS.TRACK_LENGTH_METERS / 2))),
              new SwerveModuleState(
                  0,
                  new Rotation2d(
                      Math.atan2(
                          -CONSTANTS.TRACK_WIDTH_METERS / 2, -CONSTANTS.TRACK_LENGTH_METERS / 2)))
            },
            new double[4],
            new double[4]);
    lastSetpoint =
        new SwerveSetpoint(
            new ChassisSpeeds(), swerveInputs.moduleStates, new double[4], new double[4]);

    wheelRadiusMeters = CONSTANTS.WHEEL_DIAMETER_METERS / 2;
    kTNewtonMetersPerAmp = CONSTANTS.DRIVE_MOTOR.KtNMPerAmp;
    currentVisualizationScalar =
        CONSTANTS.DRIVE_STATOR_CURRENT_LIMIT_AMPS / maxLinearSpeedMetersPerSec;
    Logger.recordOutput(
        "Swerve/Setpoint Generator/Setpoint/Current Visualization Scalar",
        currentVisualizationScalar);

    poseEstimator =
        new SwervePoseEstimator(
            setpointGenerator.getKinematics(),
            new Rotation2d(),
            getLatestModulePositions(),
            new Pose2d());
    this.visionIOs = visionIOs;
    visionInputs = new FiducialPoseEstimatorIOInputs[visionIOs.length];
    for (int i = 0; i < visionIOs.length; i++) {
      visionInputs[i] = new FiducialPoseEstimatorIOInputs();
    }

    xController = new PIDController(5, 0, 0);
    yController = new PIDController(5, 0, 0);
    yawController = new PIDController(5, 0, 0);
    yawController.enableContinuousInput(-PI, PI);
  }

  private final SwerveModuleState[] lastDriveFeedforwards =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
  private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();

  private final Alert invalidOdometryWarning =
      new Alert("Swerve: Invalid odometry data!", Alert.AlertType.kWarning);
  private final Alert outOfOrderOdometryWarning =
      new Alert(
          "Swerve: Odometry data was out of order! Expected latest data last.",
          Alert.AlertType.kWarning);

  private final ArrayList<Pose3d> tagPoses = new ArrayList<>();
  private final ArrayList<Pose3d> poseEstimates = new ArrayList<>();

  /** Whether to ignore vision inputs or cheat with the ground truth pose in sim. */
  boolean useGroundTruth = true;

  /** Whether to log vision data or not. */
  boolean visionLoggingEnabled = true;

  @Override
  public void periodic() {
    Logger.recordOutput(
        "Swerve/Setpoint Generator/Setpoint/Desired Speeds", lastSetpoint.chassisSpeeds());
    Logger.recordOutput(
        "Swerve/Setpoint Generator/Setpoint/Module Setpoints", lastSetpoint.moduleStates());
    Logger.recordOutput(
        "Swerve/Setpoint Generator/Setpoint/Steer Feedforwards",
        lastSetpoint.steerFeedforwardsRadPerSec());
    for (int i = 0; i < 4; i++) {
      lastDriveFeedforwards[i].angle = lastSetpoint.moduleStates()[i].angle;
      lastDriveFeedforwards[i].speedMetersPerSecond =
          lastSetpoint.driveFeedforwardsAmps()[i] / currentVisualizationScalar;
    }
    Logger.recordOutput(
        "Swerve/Setpoint Generator/Setpoint/Drive feedforwards", lastDriveFeedforwards);

    swerveIO.updateInputs(swerveInputs);
    Logger.processInputs("Swerve", swerveInputs);
    robotRelativeSpeeds = getRobotChassisSpeeds();
    Logger.recordOutput("Swerve/Actual Speed", robotRelativeSpeeds);

    fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, swerveInputs.gyroYaw);

    if (!poseReset) {
      var driveStdDevs = getDriveStdDevs();
      Logger.recordOutput("Swerve/Odometry/Drive Std Devs", driveStdDevs);
      poseEstimator.setDriveMeasurementStdDevs(driveStdDevs);

      double lastTimestamp = -1;
      for (var frame : swerveInputs.odometryFrames) {
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
        lastTimestamp = frame.timestamp();
      }

      if (useGroundTruth && Robot.isSimulation()) {
        if (Logger.hasReplaySource()) {
          throw new IllegalStateException("Code determinism cannot be guaranteed!");
        }
        poseEstimator.addVisionMeasurement(
            Robot.groundTruthPoseSupplier.get(),
            RobotController.getFPGATime() / 1e6,
            new double[] {.0001, .0001, .00001});
      } else {
        if (Robot.isSimulation() && !Logger.hasReplaySource()) {
          FiducialPoseEstimatorIOPhoton.VisionSim.getInstance().update();
        }

        for (int i = 0; i < visionIOs.length; i++) {
          visionIOs[i].updateInputs(visionInputs[i]);
          Logger.processInputs("Vision/" + i, visionInputs[i]);

          if (!visionInputs[i].hasNewData) {
            // If there's no new data, save some CPU
            continue;
          }

          CameraTunings tunings;
          if (i >= cameraTunings.length) {
            tunings = CameraTunings.defaults;
          } else {
            tunings = cameraTunings[i];
          }

          for (int j = 0; j < visionInputs[i].poseEstimates.length; j++) {
            var poseEstimate = visionInputs[i].poseEstimates[j];

            if (!MathUtil.isNear(0, poseEstimate.estimatedPose().getZ(), .25)) {
              continue;
            }

            var translationalStdDevs = getVisionStdDevs(poseEstimate, tunings.translational);
            var angularStdDevs = getVisionStdDevs(poseEstimate, tunings.angular);
            Logger.recordOutput(
                "Vision/Std Devs/" + i + "/" + j + "/Translational", translationalStdDevs);
            Logger.recordOutput("Vision/Std Devs/" + i + "/" + j + "/Angular", angularStdDevs);

            poseEstimator.addVisionMeasurement(
                poseEstimate.estimatedPose().toPose2d(),
                poseEstimate.timestamp(),
                new double[] {translationalStdDevs, translationalStdDevs, angularStdDevs});

            if (visionLoggingEnabled) {
              poseEstimates.add(poseEstimate.estimatedPose());
              tagPoses.addAll(Arrays.asList(poseEstimate.tagsUsed()));
            }
          }
        }
        Logger.recordOutput("Vision/All pose estimates", poseEstimates.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/All tags used", tagPoses.toArray(new Pose3d[0]));
        poseEstimates.clear();
        tagPoses.clear();
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
              swerveInputs.moduleStates[i].speedMetersPerSecond,
              swerveInputs.moduleStates[i].angle);
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
            .rotateBy(swerveInputs.gyroYaw);

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
      .0005
    };
  }

  private double getVisionStdDevs(
      FiducialPoseEstimatorIO.PoseEstimate poseEstimate, StdDevTunings tunings) {
    double distanceScoreSum = 0;
    for (var tagPose : poseEstimate.tagsUsed()) {
      var distanceMeters =
          tagPose.getTranslation().getDistance(poseEstimate.estimatedPose().getTranslation());
      distanceScoreSum += tunings.distanceScalar * distanceMeters * distanceMeters;
    }

    var tagCountDivisor = Math.pow(poseEstimate.tagsUsed().length, tunings.tagCountExponent);

    var translationalHeightScalar =
        1
            + tunings.heightScalar
                * poseEstimate.estimatedPose().getZ()
                * poseEstimate.estimatedPose().getZ();

    return (distanceScoreSum / tagCountDivisor) * translationalHeightScalar;
  }

  public Command teleopDrive(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      double translationDeadband,
      double translationExponent,
      DoubleSupplier omegaSupplier,
      double omegaDeadband,
      double omegaExponent) {
    return run(
        () -> {
          double xControl = xSupplier.getAsDouble();
          double yControl = ySupplier.getAsDouble();
          double magnitude = Math.hypot(xControl, yControl);
          if (magnitude > 1) {
            xControl /= magnitude;
            yControl /= magnitude;
          } else if (magnitude > 1e-6) {
            double scalar =
                Math.pow(
                    MathUtil.applyDeadband(magnitude, translationDeadband) / magnitude,
                    translationExponent);
            xControl *= scalar;
            yControl *= scalar;
          }

          double omegaControl = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), omegaDeadband);
          omegaControl =
              Math.pow(Math.abs(omegaControl), omegaExponent) * Math.signum(omegaControl);

          teleopDrive(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  new ChassisSpeeds(
                      xControl * maxLinearSpeedMetersPerSec,
                      yControl * maxLinearSpeedMetersPerSec,
                      omegaControl * maxAngularSpeedRadPerSec),
                  swerveInputs.gyroYaw));
        });
  }

  public Command stopInX() {
    return run(() -> {
          swerveIO.drive(stopInXSetpoint);
          lastSetpoint = stopInXSetpoint;
        })
        .ignoringDisable(true);
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
        });
  }

  private void teleopDrive(ChassisSpeeds robotRelativeSpeeds) {
    var translationalMagnitude =
        Math.hypot(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond);
    if (translationalMagnitude > maxLinearSpeedMetersPerSec) {
      robotRelativeSpeeds.vxMetersPerSecond *= maxLinearSpeedMetersPerSec / translationalMagnitude;
      robotRelativeSpeeds.vyMetersPerSecond *= maxLinearSpeedMetersPerSec / translationalMagnitude;

      translationalMagnitude = maxLinearSpeedMetersPerSec;
    }
    robotRelativeSpeeds.omegaRadiansPerSecond *=
        MathUtil.interpolate(
            1,
            angularSpeedFudgeFactor,
            MathUtil.inverseInterpolate(0, maxLinearSpeedMetersPerSec, translationalMagnitude));

    var setpoint = setpointGenerator.generateSetpoint(lastSetpoint, robotRelativeSpeeds);
    swerveIO.drive(setpoint);
    lastSetpoint = setpoint;
  }

  private void autoDrive(ChassisSpeeds speeds, Translation2d[] forceVectors) {
    var setpoint = setpointGenerator.generateSetpoint(lastSetpoint, speeds);

    for (int i = 0; i < 4; i++) {
      if (forceVectors[i].getNorm() < 1e-6) {
        setpoint.driveFeedforwardsAmps()[i] = 0;
      } else {
        setpoint.driveFeedforwardsAmps()[i] =
            forceVectors[i].getNorm()
                * forceVectors[i].getAngle().minus(setpoint.moduleStates()[i].angle).getCos()
                * wheelRadiusMeters
                / kTNewtonMetersPerAmp;
      }
    }

    swerveIO.drive(setpoint);
    lastSetpoint = setpoint;
  }

  public void followTrajectory(SwerveSample sample) {
    var feedforward =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(sample.vx, sample.vy, sample.omega), new Rotation2d(sample.heading));

    var targetPose = new Pose2d(sample.x, sample.y, new Rotation2d(sample.heading));
    var feedback =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(
                xController.calculate(
                    poseEstimator.getEstimatedPosition().getX(), targetPose.getX()),
                yController.calculate(
                    poseEstimator.getEstimatedPosition().getY(), targetPose.getY()),
                yawController.calculate(
                    poseEstimator.getEstimatedPosition().getRotation().getRadians(),
                    targetPose.getRotation().getRadians())),
            swerveInputs.gyroYaw);

    Logger.recordOutput("Choreo/Error", targetPose.minus(poseEstimator.getEstimatedPosition()));

    var forceVectors = new Translation2d[4];
    for (int i = 0; i < 4; i++) {
      forceVectors[i] =
          new Translation2d(sample.moduleForcesX()[i], sample.moduleForcesY()[i])
              .rotateBy(new Rotation2d(-sample.heading));
    }

    Logger.recordOutput("Choreo/Target Pose", targetPose);
    Logger.recordOutput("Choreo/Feedforward", feedforward);
    Logger.recordOutput("Choreo/Feedback", feedback);

    var output = feedforward.plus(feedback);
    autoDrive(output, forceVectors);
    Logger.recordOutput("Choreo/Output", output);
  }

  public Command steerCharacterize() {
    var sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(2).per(Second),
                Volts.of(6),
                Seconds.of(6),
                state -> SignalLogger.writeString("SysIDState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage -> swerveIO.steerCharacterization(voltage.baseUnitMagnitude()),
                null,
                this));

    return sequence(
        runOnce(swerveIO::initSysId),
        sysId.quasistatic(SysIdRoutine.Direction.kForward),
        waitSeconds(1),
        sysId.quasistatic(SysIdRoutine.Direction.kReverse),
        waitSeconds(1),
        sysId.dynamic(SysIdRoutine.Direction.kForward),
        waitSeconds(1),
        sysId.dynamic(SysIdRoutine.Direction.kReverse));
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
    return setpointGenerator.getKinematics().toChassisSpeeds(swerveInputs.moduleStates);
  }

  public SwerveModulePosition[] getLatestModulePositions() {
    if (swerveInputs.odometryFrames.length == 0) {
      throw new IndexOutOfBoundsException(
          "swerveInputs.odometryFrames.length was 0! This should not be possible.");
    }
    return swerveInputs.odometryFrames[swerveInputs.odometryFrames.length - 1].positions();
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  private boolean poseReset = false;

  public void resetForAuto(Pose2d pose) {
    DriverStation.reportWarning("Reset For Auto run", true);
    if (Robot.isSimulation()
        && !Logger.hasReplaySource()
        && swerveIO instanceof SwerveIOPhoenix phoenix) {
      phoenix.resetGroundTruth(pose);
    }
    var gyroAngle =
        Robot.isOnRed() ? pose.getRotation().rotateBy(Rotation2d.kPi) : pose.getRotation();
    swerveIO.resetGyro(gyroAngle);
    xController.reset();
    yController.reset();
    yawController.reset();
    poseEstimator.resetPosition(gyroAngle, getLatestModulePositions(), pose);
    swerveIO.updateInputs(swerveInputs);
    Logger.processInputs("Swerve", swerveInputs);
    poseReset = true;
  }
}
