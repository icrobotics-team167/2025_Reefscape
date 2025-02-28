// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.arm;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

public class CoralOuttakeIOPhoenix implements CoralOuttakeIO {
  private final TalonFX motor;
  private final CANrange detector;

  public CoralOuttakeIOPhoenix() {
    motor = new TalonFX(0);

    detector = new CANrange(1);
  }

  @Override
  public void updateInputs(CoralOuttakeIOInputs inputs) {
    inputs.coralIn = detector.getIsDetected(false).getValue();
  }

  @Override
  public void runVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void brake() {
    motor.setVoltage(0);
  }
}
