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
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.cotc.util.PhoenixBatchRefresher;

public class AlgaeRollersIOPhoenix implements AlgaeRollersIO {
  private final TalonFX motor;

  private final BaseStatusSignal statorSignal, supplySignal;
  private final StatusSignal<Boolean> detectedSignal;

  public AlgaeRollersIOPhoenix() {
    motor = new TalonFX(4);
    var motorConfig = new TalonFXConfiguration();
    motorConfig.CurrentLimits.StatorCurrentLimit = 60;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 20;
    motorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
    motor.getConfigurator().apply(motorConfig);

    var detector = new CANrange(3);

    var detectorConfig = new CANrangeConfiguration();
    detectorConfig.FovParams.FOVRangeX = 6.75;
    detectorConfig.FovParams.FOVRangeY = 6.75;
    detectorConfig.ProximityParams.ProximityThreshold = .15;
    detector.getConfigurator().apply(detectorConfig);

    detectedSignal = detector.getIsDetected(false);

    motor.setNeutralMode(NeutralModeValue.Brake);

    statorSignal = motor.getStatorCurrent(false);
    supplySignal = motor.getSupplyCurrent(false);

    BaseStatusSignal.setUpdateFrequencyForAll(50, statorSignal, supplySignal, detectedSignal);
    ParentDevice.optimizeBusUtilizationForAll(5, motor, detector);
    PhoenixBatchRefresher.registerRio(statorSignal, supplySignal, detectedSignal);
  }

  @Override
  public void updateInputs(AlgaeRollersIOInputs inputs) {
    inputs.currentDraws.mutateFromSignals(statorSignal, supplySignal);
    inputs.hasAlgae = detectedSignal.getValue();
  }

  @Override
  public void intake() {
    motor.setVoltage(12);
  }

  @Override
  public void hold() {
    motor.setVoltage(3);
  }

  @Override
  public void eject() {
    motor.setVoltage(-10);
  }

  @Override
  public void brake() {
    motor.stopMotor();
  }
}
