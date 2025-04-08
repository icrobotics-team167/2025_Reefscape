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
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.util.Units;
import frc.cotc.util.PhoenixBatchRefresher;

public class AlgaePivotIOPhoenix implements AlgaePivotIO {
  private final TalonFX motor;

  private final BaseStatusSignal posSignal, velSignal, statorSignal, supplySignal;

  public AlgaePivotIOPhoenix() {
    motor = new TalonFX(3);
    var encoder = new CANcoder(4);

    var config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.RotorToSensorRatio = (46.0 / 42.0) * 81;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimit = 20;
    config.Slot0.kP = 48;
    config.Slot0.kD = 1;
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    motor.getConfigurator().apply(config);

    posSignal = encoder.getAbsolutePosition(false);
    velSignal = motor.getVelocity(false);
    statorSignal = motor.getStatorCurrent(false);
    supplySignal = motor.getSupplyCurrent(false);
    BaseStatusSignal.setUpdateFrequencyForAll(50, posSignal, velSignal, statorSignal, supplySignal);
    ParentDevice.optimizeBusUtilizationForAll(5, motor, encoder);
    PhoenixBatchRefresher.registerRio(posSignal, velSignal, statorSignal, supplySignal);
  }

  @Override
  public void updateInputs(AlgaePivotIOInputs inputs) {
    inputs.posRad = Units.rotationsToRadians(posSignal.getValueAsDouble());
    inputs.velRadPerSec = Units.rotationsToRadians(velSignal.getValueAsDouble());

    inputs.currentDraws.mutateFromSignals(statorSignal, supplySignal);
  }

  private final PositionVoltage positionControlSignal = new PositionVoltage(0);

  @Override
  public void goToAngle(double angleRad) {
    motor.setControl(positionControlSignal.withPosition(Units.radiansToRotations(angleRad)));
  }
}
