// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import frc.cotc.Robot;
import frc.cotc.util.PhoenixBatchRefresher;

public class ClimberIOPhoenix implements ClimberIO {
  private final TalonFX motor;

  private final BaseStatusSignal posSignal;
  private final BaseStatusSignal statorCurrentSignal;
  private final BaseStatusSignal supplyCurrentSignal;

  public ClimberIOPhoenix() {
    var encoder = new CANcoder(12, Robot.CANIVORE_NAME);
    motor = new TalonFX(16, Robot.CANIVORE_NAME);

    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.MagnetOffset = -0.248291015625;
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoder.getConfigurator().apply(encoderConfig);

    var motorConfig = new TalonFXConfiguration();
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    motorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.27;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(5);
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    motor.getConfigurator().apply(motorConfig);

    posSignal = encoder.getAbsolutePosition(false);
    statorCurrentSignal = motor.getStatorCurrent(false);
    supplyCurrentSignal = motor.getSupplyCurrent(false);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, posSignal, statorCurrentSignal, supplyCurrentSignal);
    ParentDevice.optimizeBusUtilizationForAll(5, encoder, motor);
    PhoenixBatchRefresher.registerCanivore(posSignal, statorCurrentSignal, supplyCurrentSignal);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.posRad = Units.rotationsToRadians(posSignal.getValueAsDouble());

    inputs.currentDraws.mutateFromSignals(statorCurrentSignal, supplyCurrentSignal);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void deploy() {
    motor.setVoltage(-5);
  }

  @Override
  public void climb() {
    motor.setVoltage(12);
  }
}
