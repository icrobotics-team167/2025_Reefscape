// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.util.Units;
import frc.cotc.util.GainsCalculator;
import frc.cotc.util.PhoenixBatchRefresher;

public class AlgaeClawIOPhoenix implements AlgaeClawIO {
  private final TalonFX motor;

  private final BaseStatusSignal posSignal,velSignal,statorSignal,supplySignal;

  public AlgaeClawIOPhoenix() {
    motor = new TalonFX(3);

    var config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimit = 20;
    config.Feedback.SensorToMechanismRatio = (32.0 / 18.0) * 81;
    config.Slot0.kV = 12.0 / ((7530.0 / 60.0) / config.Feedback.SensorToMechanismRatio);
    config.Slot0.kS = 0.25;
    config.Slot0.kG = .032;
    config.Slot0.kP = 48;
    config.Slot0.kD = 1;
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    motor.getConfigurator().apply(config);

    motor.setPosition(Units.degreesToRotations(-75));

    posSignal = motor.getPosition(false);
    velSignal = motor.getVelocity(false);
    statorSignal = motor.getStatorCurrent(false);
    supplySignal = motor.getSupplyCurrent(false);
    BaseStatusSignal.setUpdateFrequencyForAll(50, posSignal, velSignal, statorSignal, supplySignal);
    motor.optimizeBusUtilization(5, .01);
    PhoenixBatchRefresher.registerRio(posSignal, velSignal, statorSignal, supplySignal);
  }

  @Override
  public void updateInputs(AlgaeClawIOInputs inputs) {
    inputs.posRad = Units.rotationsToRadians(posSignal.getValueAsDouble());
    inputs.velRadPerSec = Units.rotationsToRadians(velSignal.getValueAsDouble());

    inputs.motorCurrentDraws.mutateFromSignals(statorSignal, supplySignal);
  }

  private final PositionVoltage positionControl = new PositionVoltage(0);

  @Override
  public void setTargetPos(double angleRad) {
    motor.setControl(positionControl.withPosition(Units.radiansToRotations(angleRad)));
  }
}
