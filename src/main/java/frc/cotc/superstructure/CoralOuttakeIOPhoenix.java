// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import frc.cotc.util.PhoenixBatchRefresher;

public class CoralOuttakeIOPhoenix implements CoralOuttakeIO {
  private final TalonFX motor;

  private final BaseStatusSignal statorSignal, supplySignal, velSignal;
  private final StatusSignal<Boolean> detectedSignal;
  private final StatusSignal<Boolean> incomingSignal;

  public CoralOuttakeIOPhoenix() {
    motor = new TalonFX(0);
    var coralDetector = new CANrange(1);
    var incomingDetector = new CANrange(2);

    var motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.StatorCurrentLimit = 20;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 10;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
    motorConfig.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANrange;
    motorConfig.HardwareLimitSwitch.ForwardLimitRemoteSensorID = coralDetector.getDeviceID();
    motor.getConfigurator().apply(motorConfig);

    var detectorConfig = new CANrangeConfiguration();
    detectorConfig.FovParams.FOVRangeX = 6.75;
    detectorConfig.FovParams.FOVRangeY = 6.75;
    detectorConfig.ProximityParams.ProximityThreshold = .175;
    detectorConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
    coralDetector.getConfigurator().apply(detectorConfig);
    incomingDetector.getConfigurator().apply(detectorConfig);

    statorSignal = motor.getStatorCurrent(false);
    supplySignal = motor.getSupplyCurrent(false);
    velSignal = motor.getVelocity(false);
    detectedSignal = coralDetector.getIsDetected(false);
    incomingSignal = incomingDetector.getIsDetected(false);

    BaseStatusSignal.setUpdateFrequencyForAll(50, statorSignal, supplySignal, incomingSignal);
    detectedSignal.setUpdateFrequency(100);
    ParentDevice.optimizeBusUtilizationForAll(5, motor, coralDetector, incomingDetector);

    PhoenixBatchRefresher.registerRio(
        statorSignal, supplySignal, velSignal, detectedSignal, incomingSignal);
  }

  @Override
  public void updateInputs(CoralOuttakeIOInputs inputs) {
    inputs.hasCoral = detectedSignal.getValue();

    double maxVel = 7543.0 / 60.0;
    inputs.velocityPercent = velSignal.getValueAsDouble() / maxVel;

    inputs.currentDraws.mutateFromSignals(statorSignal, supplySignal);
  }

  private final VoltageOut idleControl = new VoltageOut(12).withIgnoreHardwareLimits(false);
  private final VoltageOut intakeControl = new VoltageOut(5).withIgnoreHardwareLimits(false);

  @Override
  public void intake() {
    motor.setControl(incomingSignal.getValue() ? intakeControl : idleControl);
  }

  private final VoltageOut fastOuttakeControl = new VoltageOut(9).withIgnoreHardwareLimits(true);

  @Override
  public void outtakeFast() {
    motor.setControl(fastOuttakeControl);
  }

  private final VoltageOut slowOuttakeControl = new VoltageOut(6).withIgnoreHardwareLimits(true);

  @Override
  public void outtakeSlow() {
    motor.setControl(slowOuttakeControl);
  }


  private final VoltageOut troughOuttakeControl = new VoltageOut(4).withIgnoreHardwareLimits(true);

  @Override
  public void outtakeTrough() {
    motor.setControl(troughOuttakeControl);
  }


  private final VoltageOut agitateControl = new VoltageOut(-12);

  @Override
  public void agitate() {
    motor.setControl(agitateControl);
  }

  @Override
  public void brake() {
    motor.stopMotor();
  }
}
