// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class CoralOuttakeIOPhoenix implements CoralOuttakeIO {
  private final TalonFX motor;
  private final CANrange detector;

  private final BaseStatusSignal statorSignal;
  private final BaseStatusSignal supplySignal;
  private final StatusSignal<Boolean> detectedSignal;

  public CoralOuttakeIOPhoenix() {
    motor = new TalonFX(15);
    detector = new CANrange(16);

    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.HardwareLimitSwitch.ForwardLimitEnable = true;
    config.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.RemoteCANrange;
    config.HardwareLimitSwitch.ForwardLimitRemoteSensorID = detector.getDeviceID();
    motor.getConfigurator().apply(config);

    statorSignal = motor.getStatorCurrent(false);
    supplySignal = motor.getSupplyCurrent(false);
    detectedSignal = detector.getIsDetected(false);

    BaseStatusSignal.setUpdateFrequencyForAll(50, statorSignal, supplySignal);
    detectedSignal.setUpdateFrequency(100);
    motor.optimizeBusUtilization(5, .1);
    detector.optimizeBusUtilization(5, .1);
  }

  @Override
  public void updateInputs(CoralOuttakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(statorSignal, supplySignal, detectedSignal);

    inputs.hasCoral = detectedSignal.getValue();

    inputs.currentDraws.mutateFromSignals(statorSignal, supplySignal);
  }

  private final VoltageOut intakeControl = new VoltageOut(12).withIgnoreHardwareLimits(false);

  @Override
  public void intake() {
    motor.setControl(intakeControl);
  }

  private final VoltageOut outtakeControl = new VoltageOut(12).withIgnoreHardwareLimits(true);

  @Override
  public void outtake() {
    motor.setControl(outtakeControl);
  }

  @Override
  public void brake() {
    motor.stopMotor();
  }
}
