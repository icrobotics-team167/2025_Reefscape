// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.cotc.Robot;
import frc.cotc.util.PhoenixBatchRefresher;

public class CoralElevatorIOPhoenix implements CoralElevatorIO {
  private final TalonFX motor;

  private final double metersPerRotation = 1;

  private final BaseStatusSignal posSignal, velSignal, statorSignal, supplySignal;

  public CoralElevatorIOPhoenix() {
    motor = new TalonFX(16, Robot.CANIVORE_NAME);

    posSignal = motor.getPosition(false);
    velSignal = motor.getVelocity(false);
    statorSignal = motor.getStatorCurrent(false);
    supplySignal = motor.getSupplyCurrent(false);

    BaseStatusSignal.setUpdateFrequencyForAll(50, posSignal, velSignal, statorSignal, supplySignal);

    motor.optimizeBusUtilization(4, .1);

    PhoenixBatchRefresher.register(posSignal, velSignal, statorSignal, supplySignal);
  }

  @Override
  public void updateInputs(CoralElevatorIOInputs inputs) {
    inputs.posMeters = posSignal.getValueAsDouble() * metersPerRotation;
    inputs.velMetersPerSec = velSignal.getValueAsDouble() * metersPerRotation;

    inputs.currentDraws.mutateFromSignals(statorSignal, supplySignal);
  }

  private final MotionMagicExpoTorqueCurrentFOC positionControl =
      new MotionMagicExpoTorqueCurrentFOC(0);

  @Override
  public void goToPos(double posMeters) {
    motor.setControl(positionControl.withPosition(posMeters / metersPerRotation));
  }

  @Override
  public void characterize(double current) {
    CoralElevatorIO.super.characterize(current);
  }
}
