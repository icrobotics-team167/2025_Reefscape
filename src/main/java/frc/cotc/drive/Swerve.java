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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.cotc.Robot;
import frc.cotc.vision.FiducialPoseEstimator;
import frc.cotc.vision.FiducialPoseEstimatorIOPhoton;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private final SwerveIO swerveIO;

  private final SwerveIO.SwerveIOInputs swerveInputs;

  private final SwerveSetpointGenerator setpointGenerator;
  private final SwerveSetpoint stopInXSetpoint;
  private SwerveSetpoint lastSetpoint;
  private final double currentVisualizationScalar;

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
            CONSTANTS.MAX_STEER_SPEEDS_RAD_PER_SEC,
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

    xController = new PIDController(10, 0, 1);
    yController = new PIDController(10, 0, 1);
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

  @Override
  public void periodic() {
    Logger.recordOutput(
        "Swerve/Setpoint Generator/Setpoint/Output Speeds", lastSetpoint.chassisSpeeds());
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

    if (Robot.isSimulation() && !Logger.hasReplaySource()) {
      FiducialPoseEstimatorIOPhoton.Sim.update();
    }

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
        yawBuffer.addSample(frame.timestamp(), frame.gyroYaw());
        lastTimestamp = frame.timestamp();
      }
      for (var fiducialPoseEstimator : fiducialPoseEstimators) {
        var poseEstimates = fiducialPoseEstimator.poll();
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
      .001
    };
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

          var commandedRobotSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  new ChassisSpeeds(
                      xControl * maxLinearSpeedMetersPerSec,
                      yControl * maxLinearSpeedMetersPerSec,
                      omegaControl * maxAngularSpeedRadPerSec),
                  swerveInputs.gyroYaw);

          var translationalMagnitude =
              Math.hypot(
                  commandedRobotSpeeds.vxMetersPerSecond, commandedRobotSpeeds.vyMetersPerSecond);
          if (translationalMagnitude > maxLinearSpeedMetersPerSec) {
            commandedRobotSpeeds.vxMetersPerSecond *=
                maxLinearSpeedMetersPerSec / translationalMagnitude;
            commandedRobotSpeeds.vyMetersPerSecond *=
                maxLinearSpeedMetersPerSec / translationalMagnitude;

            translationalMagnitude = maxLinearSpeedMetersPerSec;
          }
          commandedRobotSpeeds.omegaRadiansPerSecond *=
              MathUtil.interpolate(
                  1,
                  angularSpeedFudgeFactor,
                  MathUtil.inverseInterpolate(
                      0, maxLinearSpeedMetersPerSec, translationalMagnitude));

          var setpoint = setpointGenerator.generateSetpoint(lastSetpoint, commandedRobotSpeeds);
          swerveIO.drive(setpoint);
          lastSetpoint = setpoint;
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

  public void followChoreoTrajectory(SwerveSample sample) {
    Logger.recordOutput(
        "Choreo/Target pose", new Pose2d(sample.x, sample.y, new Rotation2d(sample.heading)));

    var feedforward = new ChassisSpeeds(sample.vx, sample.vy, sample.omega);
    var feedback =
        new ChassisSpeeds(
            xController.calculate(poseEstimator.getEstimatedPosition().getX(), sample.x),
            yController.calculate(poseEstimator.getEstimatedPosition().getY(), sample.y),
            yawController.calculate(
                poseEstimator.getEstimatedPosition().getRotation().getRadians(), sample.heading));

    var outputFieldRelative = feedforward.plus(feedback);
    var outputRobotRelative =
        ChassisSpeeds.fromFieldRelativeSpeeds(outputFieldRelative, new Rotation2d(sample.heading));

    var setpoint = setpointGenerator.generateSetpoint(lastSetpoint, outputRobotRelative);
    swerveIO.drive(setpoint);
    lastSetpoint = setpoint;
  }

  public Command followRepulsorField(Pose2d goal) {
    return sequence(
            runOnce(
                () -> {
                  repulsorFieldPlanner.setGoal(goal.getTranslation());
                  xController.reset();
                  yController.reset();
                  yawController.reset();
                }),
            run(
                () -> {
                  Logger.recordOutput("Repulsor/Goal", goal);
                  Logger.recordOutput(
                      "Repulsor/Trajectory",
                      repulsorFieldPlanner.getTrajectory(
                          poseEstimator.getEstimatedPosition().getTranslation(),
                          Robot.defaultPeriodSecs * maxLinearSpeedMetersPerSec * .8));

                  var sample =
                      repulsorFieldPlanner.sampleField(
                          poseEstimator.getEstimatedPosition().getTranslation(),
                          maxLinearSpeedMetersPerSec * .8,
                          1.5);

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

                  Logger.recordOutput(
                      "Repulsor/Error", goal.minus(poseEstimator.getEstimatedPosition()));
                  Logger.recordOutput("Repulsor/Feedforward", feedforward);
                  Logger.recordOutput("Repulsor/Feedback", feedback);

                  //                  Logger.recordOutput("Repulsor/Vector field",
                  // repulsorFieldPlanner.getArrows());

                  var outputFieldRelative = feedforward.plus(feedback);
                  var outputRobotRelative =
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          outputFieldRelative, poseEstimator.getEstimatedPosition().getRotation());

                  var setpoint =
                      setpointGenerator.generateSetpoint(lastSetpoint, outputRobotRelative);
                  swerveIO.drive(setpoint);
                  lastSetpoint = setpoint;
                }))
        .until(
            () -> {
              var error = goal.minus(poseEstimator.getEstimatedPosition());
              return error.getTranslation().getNorm() < .01
                  && Math.abs(error.getRotation().getDegrees()) < 5;
            });
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
    swerveIO.updateInputs(swerveInputs);
    Logger.processInputs("Swerve", swerveInputs);
    poseReset = true;
  }
}
