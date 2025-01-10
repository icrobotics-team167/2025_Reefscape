// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.cotc.Robot;
import frc.cotc.util.FOCArmSim;
import frc.cotc.util.FOCMotorSim;
import frc.cotc.util.PhoenixBatchRefresher;

public class AlgaeClawIOPhoenix implements AlgaeClawIO {
  private final TalonFX pivotMotor;
  private final TalonFX intakeMotor;

  private final BaseStatusSignal pivotAngle,
      pivotVel,
      wheelPos,
      wheelVel,
      pivotStator,
      pivotSupply,
      intakeStator,
      intakeSupply;

  private final double pivotGearRatio = 100;

  private final DCMotor pivotMotorModel = DCMotor.getKrakenX60Foc(1).withReduction(pivotGearRatio);
  private final double armMassKg = Units.lbsToKilograms(5);
  private final double armCoMRadius = .5;

  public AlgaeClawIOPhoenix() {
    pivotMotor = new TalonFX(16, Robot.CANIVORE_NAME);
    var pivotEncoder = new CANcoder(17, Robot.CANIVORE_NAME);
    intakeMotor = new TalonFX(18, Robot.CANIVORE_NAME);

    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.CurrentLimits.StatorCurrentLimit = 80;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = 60;
    pivotConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    pivotConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
    pivotConfig.Feedback.RotorToSensorRatio = pivotGearRatio;
    pivotConfig.Feedback.SensorToMechanismRatio = 1;
    pivotConfig.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    pivotConfig.Audio.AllowMusicDurDisable = true;

    var intakeConfig = new TalonFXConfiguration();
    intakeConfig.CurrentLimits.StatorCurrentLimit = 60;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = 40;
    intakeConfig.CurrentLimits.SupplyCurrentLowerLimit = 10;
    intakeConfig.CurrentLimits.SupplyCurrentLowerTime = 1;

    // TODO: See if this math makes sense?
    pivotConfig.Slot0.kA =
        pivotMotorModel.getCurrent(
            Units.radiansToRotations(SingleJointedArmSim.estimateMOI(armCoMRadius, armMassKg)));
    pivotConfig.Slot0.kG = pivotMotorModel.getCurrent(armMassKg * 9.81 * armCoMRadius);
    pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    pivotConfig.MotionMagic.MotionMagicExpo_kV =
        12 / Units.radiansToRotations(pivotMotorModel.freeSpeedRadPerSec);
    if (Robot.isReal()) {
      pivotConfig.Slot0.kP = 0;
    } else {
      pivotConfig.Slot0.kP = 1;
      pivotConfig.MotionMagic.MotionMagicExpo_kA = .075;
    }

    pivotMotor.getConfigurator().apply(pivotConfig);

    pivotAngle = pivotEncoder.getAbsolutePosition(false);
    pivotVel = pivotMotor.getVelocity(false);
    wheelPos = intakeMotor.getPosition(false);
    wheelVel = intakeMotor.getVelocity(false);
    pivotStator = pivotMotor.getStatorCurrent(false);
    pivotSupply = pivotMotor.getSupplyCurrent(false);
    intakeStator = intakeMotor.getStatorCurrent(false);
    intakeSupply = intakeMotor.getSupplyCurrent(false);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        pivotAngle,
        pivotVel,
        wheelPos,
        wheelVel,
        pivotStator,
        pivotSupply,
        intakeStator,
        intakeSupply);
    ParentDevice.optimizeBusUtilizationForAll(4, pivotMotor, pivotEncoder, intakeMotor);
    PhoenixBatchRefresher.register(
        pivotAngle,
        pivotVel,
        wheelPos,
        wheelVel,
        pivotStator,
        pivotSupply,
        intakeStator,
        intakeSupply);

    if (Robot.isSimulation()) {
      initSim(pivotMotor, pivotEncoder, intakeMotor);
      simNotifier.startPeriodic(simDt);
    }
  }

  @Override
  public void updateInputs(AlgaeClawIOInputs inputs) {
    inputs.pivotAngleRad = Units.rotationsToRadians(pivotAngle.getValueAsDouble());
    inputs.pivotVelRadPerSec = Units.rotationsToRadians(pivotVel.getValueAsDouble());
    inputs.wheelPosMeters = wheelPos.getValueAsDouble() * wheelCircumferenceMeters;
    inputs.wheelVelMetersPerSec = wheelVel.getValueAsDouble() * wheelCircumferenceMeters;
    inputs.pivotCurrentDraws.mutateFromSignals(pivotStator, pivotSupply);
    inputs.wheelCurrentDraws.mutateFromSignals(intakeStator, intakeSupply);
  }

  private final MotionMagicExpoTorqueCurrentFOC positionControl =
      new MotionMagicExpoTorqueCurrentFOC(0);

  @Override
  public void setPivotPos(double angleRad) {
    pivotMotor.setControl(positionControl.withPosition(Units.radiansToRotations(angleRad)));
  }

  private final double wheelCircumferenceMeters = Units.inchesToMeters(4) * Math.PI;
  private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0);

  @Override
  public void runIntake(double speedMetersPerSec) {
    intakeMotor.setControl(
        velocityControl.withVelocity(speedMetersPerSec / wheelCircumferenceMeters));
  }

  private final TorqueCurrentFOC characterizationControl = new TorqueCurrentFOC(0);

  @Override
  public void characterizePivot(double amps) {
    pivotMotor.setControl(characterizationControl.withOutput(amps));
  }

  @Override
  public void characterizeIntake(double amps) {
    intakeMotor.setControl(characterizationControl.withOutput(0));
  }

  @Override
  public void initSysId() {
    BaseStatusSignal.setUpdateFrequencyForAll(
        250,
        pivotAngle,
        pivotVel,
        wheelPos,
        wheelVel,
        pivotMotor.getTorqueCurrent(false),
        intakeMotor.getTorqueCurrent(false));
  }

  private FOCArmSim armSim;
  private FOCMotorSim intakeSim;
  private TalonFXSimState pivotMotorSim;
  private CANcoderSimState pivotEncoderSim;
  private TalonFXSimState intakeMotorSim;

  private final double simDt = .001;
  private Notifier simNotifier;

  private void initSim(TalonFX pivotMotor, CANcoder pivotEncoder, TalonFX intakeMotor) {
    armSim =
        new FOCArmSim(
            pivotMotorModel,
            armMassKg,
            armCoMRadius,
            Units.degreesToRadians(-90),
            Units.degreesToRadians(90),
            Units.degreesToRadians(-90));

    intakeSim = new FOCMotorSim(DCMotor.getKrakenX60Foc(1), .005);

    pivotMotorSim = pivotMotor.getSimState();
    pivotEncoderSim = pivotEncoder.getSimState();
    intakeMotorSim = intakeMotor.getSimState();

    simNotifier = new Notifier(this::tickSim);
  }

  private void tickSim() {
    armSim.tick(pivotMotorSim.getTorqueCurrent(), simDt);

    intakeSim.tick(intakeMotorSim.getTorqueCurrent(), simDt);

    pivotEncoderSim.setRawPosition(Units.radiansToRotations(armSim.getPosRad()));
    pivotEncoderSim.setVelocity(Units.radiansToRotations(armSim.getVelRadPerSec()));

    pivotMotorSim.setRawRotorPosition(
        Units.radiansToRotations(armSim.getPosRad()) * pivotGearRatio);
    pivotMotorSim.setRotorVelocity(
        Units.radiansToRotations(armSim.getVelRadPerSec()) * pivotGearRatio);
    pivotMotorSim.setRotorAcceleration(
        Units.radiansToRotations(armSim.getAccelRadPerSecSquared()) * pivotGearRatio);

    intakeMotorSim.setRawRotorPosition(Units.radiansToRotations(intakeSim.getPos()));
    intakeMotorSim.setRotorVelocity(Units.radiansToRotations(intakeSim.getVel()));
    intakeMotorSim.setRotorAcceleration(Units.radiansToRotations(intakeSim.getAccel()));
  }
}
