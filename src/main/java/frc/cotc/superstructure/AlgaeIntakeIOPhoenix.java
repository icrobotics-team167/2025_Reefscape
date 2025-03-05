// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.cotc.util.PhoenixBatchRefresher;

public class AlgaeIntakeIOPhoenix implements AlgaeIntakeIO {
  private final TalonFX motor;

  private final BaseStatusSignal statorSignal, supplySignal;

  public AlgaeIntakeIOPhoenix() {
    motor = new TalonFX(4);

    motor.setNeutralMode(NeutralModeValue.Brake);

    statorSignal = motor.getStatorCurrent(false);
    supplySignal = motor.getSupplyCurrent(false);

    BaseStatusSignal.setUpdateFrequencyForAll(50, statorSignal, supplySignal);
    motor.optimizeBusUtilization(5, .01);
    PhoenixBatchRefresher.registerRio(statorSignal, supplySignal);
  }

  @Override
  public void updateInputs(AlgaeIntakeIOInputs inputs) {
    inputs.currentDraws.mutateFromSignals(statorSignal, supplySignal);
  }

  @Override
  public void intake() {
    motor.setVoltage(-2);
  }

  @Override
  public void outtake() {
    motor.setVoltage(1.5);
  }

  @Override
  public void brake() {
    motor.stopMotor();
  }
}
