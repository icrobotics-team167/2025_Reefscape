// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.cotc.Robot;
import frc.cotc.util.PhoenixBatchRefresher;

public class CoralOuttakeIOPhoenix implements CoralOuttakeIO {
  private final TalonFX motor;

  private final BaseStatusSignal statorSignal;
  private final BaseStatusSignal supplySignal;

  public CoralOuttakeIOPhoenix() {
    motor = new TalonFX(15, Robot.CANIVORE_NAME);

    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor.getConfigurator().apply(config);

    statorSignal = motor.getStatorCurrent(false);
    supplySignal = motor.getSupplyCurrent(false);

    BaseStatusSignal.setUpdateFrequencyForAll(50, statorSignal, supplySignal);
    PhoenixBatchRefresher.register(statorSignal, supplySignal);
    motor.optimizeBusUtilization(5, .1);
  }

  @Override
  public void updateInputs(CoralOuttakeIOInputs inputs) {
    inputs.hasCoral = false; // TODO: Implement

    inputs.currentDraws.mutateFromSignals(statorSignal, supplySignal);
  }

  @Override
  public void run() {
    motor.setVoltage(12);
  }

  @Override
  public void brake() {
    motor.stopMotor();
  }
}
