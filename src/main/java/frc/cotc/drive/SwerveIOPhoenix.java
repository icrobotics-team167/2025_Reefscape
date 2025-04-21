// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.drive;

import static frc.cotc.drive.SwerveSetpointGenerator.SwerveSetpoint;
import static java.lang.Math.PI;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.cotc.Constants;
import frc.cotc.Robot;
import frc.cotc.util.FOCMotorSim;
import frc.cotc.util.PhoenixBatchRefresher;

public class SwerveIOPhoenix implements SwerveIO {
  private static final SwerveModuleConstantsAutoLogged CONSTANTS;
  private static final double DRIVE_GEAR_RATIO;
  private static final double[] STEER_GEAR_RATIOS;
  private static final double WHEEL_CIRCUMFERENCE_METERS;

  static {
    CONSTANTS = new SwerveModuleConstantsAutoLogged();

    CONSTANTS.TRACK_WIDTH_METERS = Constants.FRAME_WIDTH_METERS - Units.inchesToMeters(2.625 * 2);
    CONSTANTS.TRACK_LENGTH_METERS = Constants.FRAME_LENGTH_METERS - Units.inchesToMeters(2.625 * 2);
    CONSTANTS.WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
    WHEEL_CIRCUMFERENCE_METERS = CONSTANTS.WHEEL_DIAMETER_METERS * PI;

    DRIVE_GEAR_RATIO = (50.0 / 16.0) * (19.0 / 25.0) * (45.0 / 15.0);
    CONSTANTS.DRIVE_MOTOR = DCMotor.getKrakenX60Foc(1).withReduction(DRIVE_GEAR_RATIO);

    var MK4N_STEER_GEAR_RATIO = 18.75;
    var MK4I_STEER_GEAR_RATIO = 150.0 / 7;
    STEER_GEAR_RATIOS =
        new double[] {
          MK4N_STEER_GEAR_RATIO, MK4N_STEER_GEAR_RATIO, MK4I_STEER_GEAR_RATIO, MK4I_STEER_GEAR_RATIO
        };

    var STEER_MOTOR_MAX_SPEED = Units.rotationsPerMinuteToRadiansPerSecond(6000);

    CONSTANTS.MAX_STEER_SPEEDS_RAD_PER_SEC =
        new double[] {
          STEER_MOTOR_MAX_SPEED / STEER_GEAR_RATIOS[0],
          STEER_MOTOR_MAX_SPEED / STEER_GEAR_RATIOS[1],
          STEER_MOTOR_MAX_SPEED / STEER_GEAR_RATIOS[2],
          STEER_MOTOR_MAX_SPEED / STEER_GEAR_RATIOS[3]
        };

    CONSTANTS.MASS_KG = Units.lbsToKilograms(115 + 17 + 15);

    CONSTANTS.MOI_KG_METERS_SQUARED =
        CONSTANTS.MASS_KG
            * Math.hypot(CONSTANTS.TRACK_LENGTH_METERS / 2, CONSTANTS.TRACK_WIDTH_METERS / 2)
            * Math.hypot(CONSTANTS.TRACK_LENGTH_METERS / 2, CONSTANTS.TRACK_WIDTH_METERS / 2);

    CONSTANTS.ANGULAR_SPEED_FUDGING = .6;

    CONSTANTS.SLIP_CURRENT_AMPS = 90;
  }

  private final Module[] modules = new Module[4];
  private final BaseStatusSignal[] signals = new BaseStatusSignal[34];

  private final OdometryThread odometryThread;
  private final Pigeon2 gyro;

  private SimThread simThread;

  public SwerveIOPhoenix(boolean isCompBot) {
    var devices = new ParentDevice[13];
    var lowFreqSignals = new BaseStatusSignal[20];
    for (int i = 0; i < 4; i++) {
      modules[i] = new Module(i, isCompBot);
      signals[i * 8] = modules[i].driveMotor.getVelocity(false);
      signals[i * 8 + 1] = modules[i].encoder.getAbsolutePosition(false);
      signals[i * 8 + 2] = modules[i].steerMotor.getVelocity(false);
      signals[i * 8 + 3] = modules[i].driveMotor.getAcceleration(false);
      signals[i * 8 + 4] = modules[i].driveMotor.getStatorCurrent(false);
      signals[i * 8 + 5] = modules[i].driveMotor.getSupplyCurrent(false);
      signals[i * 8 + 6] = modules[i].steerMotor.getStatorCurrent(false);
      signals[i * 8 + 7] = modules[i].steerMotor.getSupplyCurrent(false);

      System.arraycopy(modules[i].getDevices(), 0, devices, i * 3, 3);

      System.arraycopy(signals, i * 8 + 3, lowFreqSignals, i * 5, 5);
    }
    gyro = new Pigeon2(12, Robot.CANIVORE_NAME);
    devices[12] = gyro;
    signals[32] = gyro.getYaw(false);
    signals[33] = gyro.getAngularVelocityZWorld(false);

    PhoenixBatchRefresher.registerCanivore(signals);

    odometryThread = new OdometryThread(modules, gyro, 250);

    BaseStatusSignal.setUpdateFrequencyForAll(50, lowFreqSignals);

    ParentDevice.optimizeBusUtilizationForAll(5, devices);

    if (Robot.isSimulation()) {
      simThread = new SimThread(modules, gyro);
      simThread.start();
    }
    odometryThread.start();
  }

  @Override
  public void updateInputs(SwerveIOInputs inputs) {
    for (int i = 0; i < 4; i++) {
      inputs.moduleStates[i] = getCurrentState(i);

      var driveStator = signals[i * 8 + 4];
      var driveSupply = signals[i * 8 + 5];
      var steerStator = signals[i * 8 + 6];
      var steerSupply = signals[i * 8 + 7];
      inputs.driveMotorCurrents[i].mutateFromSignals(driveStator, driveSupply);
      inputs.steerMotorCurrents[i].mutateFromSignals(steerStator, steerSupply);
    }
    inputs.gyroYaw =
        Rotation2d.fromDegrees(
            BaseStatusSignal.getLatencyCompensatedValueAsDouble(signals[32], signals[33]));

    var odometryData = odometryThread.poll();

    if (odometryData.length > 0) {
      inputs.odometryFrames = odometryData;
    } else {
      // If the odometry thread doesn't have anything, fall back to getting the data directly
      // Slower but guaranteed
      var positions = new SwerveModulePosition[4];
      for (int i = 0; i < 4; i++) {
        positions[i] = modules[i].getPosition();
      }
      inputs.odometryFrames =
          new OdometryFrame[] {
            new OdometryFrame(
                positions,
                Rotation2d.fromDegrees(
                    BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                        gyro.getYaw(), gyro.getAngularVelocityZWorld())),
                RobotController.getFPGATime() / 1e6)
          };
    }
  }

  private SwerveModuleState getCurrentState(int id) {
    return new SwerveModuleState(
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(signals[id * 8], signals[id * 8 + 3])
            * WHEEL_CIRCUMFERENCE_METERS,
        Rotation2d.fromRotations(
            BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                signals[id * 8 + 1], signals[id * 8 + 2])));
  }

  @Override
  public void drive(SwerveSetpoint setpoint) {
    for (int i = 0; i < 4; i++) {
      modules[i].run(
          setpoint.moduleStates()[i],
          BaseStatusSignal.getLatencyCompensatedValueAsDouble(signals[i * 8], signals[i * 8 + 3])
              * WHEEL_CIRCUMFERENCE_METERS,
          Units.radiansToRotations(setpoint.steerFeedforwardsRadPerSec()[i]),
          setpoint.driveFeedforwardsAmps()[i]);
    }
  }

  @Override
  public void stop(Rotation2d[] angles) {
    for (int i = 0; i < 4; i++) {
      modules[i].brake(angles[i]);
    }
  }

  @Override
  public void testSlipCurrent(double amps) {
    for (var module : modules) {
      module.testSlipCurrent(amps);
    }
  }

  @Override
  public void resetGyro(Rotation2d newYaw) {
    gyro.setYaw(newYaw.getDegrees());
  }

  @Override
  public SwerveModuleConstantsAutoLogged getConstants() {
    return CONSTANTS;
  }

  private static class Module {
    final TalonFX driveMotor;
    final TalonFX steerMotor;
    final CANcoder encoder;

    @SuppressWarnings("DuplicateBranchesInSwitch")
    public Module(int id, boolean isCompBot) {
      driveMotor = new TalonFX(id * 3, Robot.CANIVORE_NAME);
      steerMotor = new TalonFX(id * 3 + 1, Robot.CANIVORE_NAME);
      encoder = new CANcoder(id * 3 + 2, Robot.CANIVORE_NAME);

      var driveConfig = new TalonFXConfiguration();
      driveConfig.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;
      driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      driveConfig.CurrentLimits.StatorCurrentLimit = CONSTANTS.SLIP_CURRENT_AMPS + 10;
      driveConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
      driveConfig.Audio.AllowMusicDurDisable = true;

      var steerConfig = new TalonFXConfiguration();
      steerConfig.Feedback.SensorToMechanismRatio = 1;
      steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
      steerConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
      steerConfig.Feedback.RotorToSensorRatio = STEER_GEAR_RATIOS[id];
      steerConfig.Feedback.SensorToMechanismRatio = 1;
      steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      steerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
      steerConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
      steerConfig.CurrentLimits.SupplyCurrentLimit = 20;
      steerConfig.CurrentLimits.SupplyCurrentLowerLimit = 10;
      steerConfig.CurrentLimits.SupplyCurrentLowerTime = .5;
      steerConfig.Audio.AllowMusicDurDisable = true;
      steerConfig.Slot0.kV =
          12 / Units.radiansToRotations(CONSTANTS.MAX_STEER_SPEEDS_RAD_PER_SEC[id]);

      var encoderConfig = new CANcoderConfiguration();
      encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

      if (Robot.isReal()) {
        if (isCompBot) {
          driveConfig.Slot0.kV = 2;
          driveConfig.Slot0.kP = 24;
          driveConfig.Slot0.kS = 1;

          switch (id) {
            case 0, 1 -> {
              steerConfig.Slot0.kP = 80;
              steerConfig.Slot0.kD = 0.1;
            }
            case 2, 3 -> {
              steerConfig.Slot0.kP = 80;
              steerConfig.Slot0.kD = 0.1;
            }
          }

          switch (id) {
            case 0 -> encoderConfig.MagnetSensor.MagnetOffset = 0.414794921875;
            case 1 -> encoderConfig.MagnetSensor.MagnetOffset = 0.21630859375;
            case 2 -> encoderConfig.MagnetSensor.MagnetOffset = 0.48974609375;
            case 3 -> encoderConfig.MagnetSensor.MagnetOffset = 0.443359375;
          }
        } else {
          driveConfig.Slot0.kV = 2.25;
          driveConfig.Slot0.kP = 15;

          steerConfig.Slot0.kP = 80;
          steerConfig.Slot0.kD = 0.1;

          switch (id) {
            case 0 -> encoderConfig.MagnetSensor.MagnetOffset = 0.295166015625;
            case 1 -> encoderConfig.MagnetSensor.MagnetOffset = 0.204345703125;
            case 2 -> encoderConfig.MagnetSensor.MagnetOffset = -0.2734375;
            case 3 -> encoderConfig.MagnetSensor.MagnetOffset = -0.08984375;
          }
        }
      } else {
        driveConfig.Slot0.kP = 15;
        steerConfig.Slot0.kP = 80;
        steerConfig.Slot0.kD = .1;
      }

      driveMotor.getConfigurator().apply(driveConfig);
      steerMotor.getConfigurator().apply(steerConfig);
      encoder.getConfigurator().apply(encoderConfig);
    }

    private final VelocityTorqueCurrentFOC driveControlRequest =
        new VelocityTorqueCurrentFOC(0).withOverrideCoastDurNeutral(true);
    private final PositionVoltage steerControlRequest = new PositionVoltage(0).withEnableFOC(false);
    private final StaticBrake brakeControlRequest = new StaticBrake();

    void run(
        SwerveModuleState desiredState,
        double currentVel,
        double steerFeedforwardRotPerSec,
        double forceFeedforwardAmps) {
      if (MathUtil.isNear(0, desiredState.speedMetersPerSecond, .05)
          && MathUtil.isNear(0, forceFeedforwardAmps, .05)
          && MathUtil.isNear(0, currentVel, .05)) {
        driveMotor.setControl(brakeControlRequest);
      } else {
        driveMotor.setControl(
            driveControlRequest
                .withVelocity(desiredState.speedMetersPerSecond / WHEEL_CIRCUMFERENCE_METERS)
                .withFeedForward(forceFeedforwardAmps));
      }
      steerMotor.setControl(
          steerControlRequest
              .withPosition(desiredState.angle.getRotations())
              .withVelocity(steerFeedforwardRotPerSec));
    }

    void brake(Rotation2d angle) {
      driveMotor.setControl(brakeControlRequest);
      steerMotor.setControl(steerControlRequest.withPosition(angle.getRotations()));
    }

    private final TorqueCurrentFOC driveCharacterization = new TorqueCurrentFOC(0);

    void testSlipCurrent(double amps) {
      steerMotor.setControl(steerControlRequest.withPosition(0));
      driveMotor.setControl(driveCharacterization.withOutput(amps));
    }

    SwerveModulePosition getPosition() {
      return new SwerveModulePosition(
          driveMotor.getPosition().getValueAsDouble() * WHEEL_CIRCUMFERENCE_METERS,
          Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble()));
    }

    ParentDevice[] getDevices() {
      return new ParentDevice[] {driveMotor, steerMotor, encoder};
    }
  }

  private static class OdometryThread extends Thread {
    final ModuleSignals[] moduleSignals = new ModuleSignals[4];

    final BaseStatusSignal[] signals = new BaseStatusSignal[18];

    final CircularBuffer<OdometryFrame> frameBuffer;
    final double FREQUENCY;

    OdometryThread(Module[] modules, Pigeon2 gyro, double frequency) {
      for (int i = 0; i < 4; i++) {
        moduleSignals[i] = ModuleSignals.fromModule(modules[i]);
        System.arraycopy(moduleSignals[i].asArray(), 0, signals, i * 4, 4);
      }

      signals[16] = gyro.getYaw();
      signals[17] = gyro.getAngularVelocityZWorld();

      BaseStatusSignal.setUpdateFrequencyForAll(frequency, signals);

      setDaemon(true);
      setName("Phoenix Odometry Thread");
      Threads.setCurrentThreadPriority(true, 2);

      FREQUENCY = frequency;
      frameBuffer = new CircularBuffer<>((int) Math.round(2 * FREQUENCY * Robot.defaultPeriodSecs));
    }

    @Override
    public void run() {
      //noinspection InfiniteLoopStatement
      while (true) {
        if (BaseStatusSignal.waitForAll(2.0 / FREQUENCY, signals) != StatusCode.OK) {
          continue;
        }

        double latencySum = 0;
        for (var signal : signals) {
          latencySum += signal.getTimestamp().getLatency();
        }
        var frame =
            new OdometryFrame(
                new SwerveModulePosition[] {
                  moduleSignals[0].poll(),
                  moduleSignals[1].poll(),
                  moduleSignals[2].poll(),
                  moduleSignals[3].poll()
                },
                Rotation2d.fromDegrees(
                    BaseStatusSignal.getLatencyCompensatedValueAsDouble(signals[16], signals[17])),
                (RobotController.getFPGATime() / 1e6) - (latencySum / signals.length));
        synchronized (frameBuffer) {
          frameBuffer.addLast(frame);
        }
      }
    }

    OdometryFrame[] poll() {
      OdometryFrame[] retFrames;
      synchronized (frameBuffer) {
        retFrames = new OdometryFrame[frameBuffer.size()];
        for (int i = 0; i < frameBuffer.size(); i++) {
          retFrames[i] = frameBuffer.get(i);
        }
        frameBuffer.clear();
      }
      return retFrames;
    }

    record ModuleSignals(
        BaseStatusSignal drivePos,
        BaseStatusSignal driveVel,
        BaseStatusSignal steerPos,
        BaseStatusSignal steerVel) {
      BaseStatusSignal[] asArray() {
        return new BaseStatusSignal[] {drivePos, driveVel, steerPos, steerVel};
      }

      SwerveModulePosition poll() {
        return new SwerveModulePosition(
            BaseStatusSignal.getLatencyCompensatedValueAsDouble(drivePos, driveVel)
                * WHEEL_CIRCUMFERENCE_METERS,
            Rotation2d.fromRotations(
                BaseStatusSignal.getLatencyCompensatedValueAsDouble(steerPos, steerVel)));
      }

      static ModuleSignals fromModule(Module module) {
        return new ModuleSignals(
            module.driveMotor.getPosition(),
            module.driveMotor.getVelocity(),
            module.encoder.getAbsolutePosition(),
            module.steerMotor.getVelocity());
      }
    }
  }

  protected void resetGroundTruth(Pose2d pose) {
    simThread.resetGroundTruthPose(pose);
  }

  private static class SimThread {
    final SimModule[] simModules = new SimModule[4];
    final Pigeon2SimState gyroSimState;
    final Notifier notifier;

    SimThread(Module[] modules, Pigeon2 gyro) {
      for (int i = 0; i < 4; i++) {
        simModules[i] = new SimModule(modules[i], STEER_GEAR_RATIOS[i]);
      }
      gyroSimState = gyro.getSimState();

      notifier = new Notifier(this::run);
      notifier.setName("Phoenix Sim Thread");
      groundTruthOdometry =
          new SwerveDriveOdometry(
              kinematics,
              new Rotation2d(),
              new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
              },
              new Pose2d(7, 2, Rotation2d.fromDegrees(120)));

      Robot.groundTruthPoseSupplier =
          () -> {
            synchronized (groundTruthOdometry) {
              return groundTruthOdometry.getPoseMeters();
            }
          };
      Robot.groundTruthSpeedSupplier =
          () -> {
            synchronized (simModules) {
              return kinematics.toChassisSpeeds(
                  simModules[0].getModuleState(),
                  simModules[1].getModuleState(),
                  simModules[2].getModuleState(),
                  simModules[3].getModuleState());
            }
          };
    }

    void start() {
      for (int i = 0; i < 4; i++) {
        var module = simModules[i];
        module.steerSim.setState((Math.random() * 2 - 1) * PI, 0);
        module.steerMotorSim.setRawRotorPosition(
            module.steerSim.getAngularPositionRotations() * STEER_GEAR_RATIOS[i]);
        module.encoderSim.setRawPosition(module.steerSim.getAngularPositionRotations());
      }

      double frequencySeconds = 1.0 / 1000;

      // Minus one iteration to prevent divide by 0 errors later
      lastTime = (RobotController.getFPGATime() / 1e6) - frequencySeconds;

      notifier.startPeriodic(frequencySeconds);
    }

    void resetGroundTruthPose(Pose2d pose) {
      synchronized (groundTruthOdometry) {
        groundTruthOdometry.resetPose(pose);
      }
    }

    private final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            new Translation2d(CONSTANTS.TRACK_LENGTH_METERS / 2, CONSTANTS.TRACK_WIDTH_METERS / 2),
            new Translation2d(CONSTANTS.TRACK_LENGTH_METERS / 2, -CONSTANTS.TRACK_WIDTH_METERS / 2),
            new Translation2d(-CONSTANTS.TRACK_LENGTH_METERS / 2, CONSTANTS.TRACK_WIDTH_METERS / 2),
            new Translation2d(
                -CONSTANTS.TRACK_LENGTH_METERS / 2, -CONSTANTS.TRACK_WIDTH_METERS / 2));
    private final SwerveDriveOdometry groundTruthOdometry;
    private double yawDeg = 0;
    private double filteredCurrentDraw = 0;
    private double lastTime;

    private void run() {
      double currentTime = RobotController.getFPGATime() / 1e6;
      double dt = currentTime - lastTime;

      double voltage = 12.3 - (.018 * filteredCurrentDraw);
      Robot.simVoltage = voltage;
      double instantaneousCurrentDraw = 0;
      SwerveModuleState[] moduleStates = new SwerveModuleState[4];
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      for (int i = 0; i < 4; i++) {
        instantaneousCurrentDraw += simModules[i].run(dt, voltage);
        moduleStates[i] = simModules[i].getModuleState();
        modulePositions[i] = simModules[i].getModulePosition();
      }
      // On a real battery, the battery's internal capacitance absorbs large spikes in current,
      // but accurately simulating that is a PITA, so in order to simulate capacitance, the
      // current draw is run through a simple low pass filter to smooth out the current draw.
      // Without this, large current spikes can trigger the TalonFX over-voltage protection.
      filteredCurrentDraw += (instantaneousCurrentDraw - filteredCurrentDraw) * (dt * 12.5);

      yawDeg +=
          Units.radiansToDegrees(
              kinematics.toChassisSpeeds(moduleStates).omegaRadiansPerSecond * dt);
      gyroSimState.setRawYaw(yawDeg);

      synchronized (groundTruthOdometry) {
        groundTruthOdometry.update(Rotation2d.fromDegrees(yawDeg), modulePositions);
      }

      lastTime = currentTime;

      Thread.yield();
    }

    private static class SimModule {
      final TalonFXSimState driveMotorSim;
      final TalonFXSimState steerMotorSim;
      final CANcoderSimState encoderSim;

      final FOCMotorSim driveWheelSim;
      final DCMotorSim steerSim;

      final double STEER_GEAR_RATIO;

      SimModule(Module module, double STEER_GEAR_RATIO) {
        driveMotorSim = module.driveMotor.getSimState();
        steerMotorSim = module.steerMotor.getSimState();
        encoderSim = module.encoder.getSimState();

        steerMotorSim.Orientation = ChassisReference.Clockwise_Positive;
        encoderSim.Orientation = ChassisReference.CounterClockwise_Positive;

        driveWheelSim =
            new FOCMotorSim(
                CONSTANTS.DRIVE_MOTOR,
                CONSTANTS.MASS_KG
                    * (CONSTANTS.WHEEL_DIAMETER_METERS / 2)
                    * (CONSTANTS.WHEEL_DIAMETER_METERS / 2)
                    / 4);
        this.STEER_GEAR_RATIO = STEER_GEAR_RATIO;
        steerSim =
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60(1), .0025, STEER_GEAR_RATIO),
                DCMotor.getKrakenX60(1));
      }

      private double lastSteerRotorVel = 0;

      double run(double dt, double voltage) {
        driveMotorSim.setSupplyVoltage(voltage);
        steerMotorSim.setSupplyVoltage(voltage);

        // Update drive sim
        driveWheelSim.tick(driveMotorSim.getTorqueCurrent(), dt);

        driveMotorSim.setRawRotorPosition(
            Units.radiansToRotations(driveWheelSim.getPos()) * DRIVE_GEAR_RATIO);
        driveMotorSim.setRotorVelocity(
            Units.radiansToRotations(driveWheelSim.getVel()) * DRIVE_GEAR_RATIO);
        driveMotorSim.setRotorAcceleration(
            Units.radiansToRotations(driveWheelSim.getAccel()) * DRIVE_GEAR_RATIO);

        // Update steer sim
        steerSim.setInputVoltage(steerMotorSim.getMotorVoltage());
        steerSim.update(dt);

        double steerRotorVel = steerSim.getAngularVelocityRPM() / 60.0 * STEER_GEAR_RATIO;

        steerMotorSim.setRawRotorPosition(
            steerSim.getAngularPositionRotations() * STEER_GEAR_RATIO);
        steerMotorSim.setRotorVelocity(steerRotorVel);
        encoderSim.setRawPosition(steerSim.getAngularPositionRotations());
        encoderSim.setVelocity(steerSim.getAngularVelocityRPM() / 60.0);

        steerMotorSim.setRotorAcceleration((lastSteerRotorVel - steerRotorVel) / dt);

        lastSteerRotorVel = steerRotorVel;

        return driveMotorSim.getSupplyCurrent() + steerMotorSim.getSupplyCurrent();
      }

      SwerveModuleState getModuleState() {
        return new SwerveModuleState(
            driveWheelSim.getVel() * CONSTANTS.WHEEL_DIAMETER_METERS / 2,
            new Rotation2d(steerSim.getAngularPositionRad()));
      }

      SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            driveWheelSim.getPos() * CONSTANTS.WHEEL_DIAMETER_METERS / 2,
            new Rotation2d(steerSim.getAngularPositionRad()));
      }
    }
  }
}
