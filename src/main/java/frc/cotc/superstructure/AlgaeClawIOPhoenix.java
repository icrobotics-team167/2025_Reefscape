// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.cotc.Robot;

public class AlgaeClawIOPhoenix implements AlgaeClawIO {
  private final TalonFX pivotMotor;
  private final CANcoder pivotEncoder;
  private final TalonFX intakeMotor;

  public AlgaeClawIOPhoenix() {
    pivotMotor = new TalonFX(16, Robot.CANIVORE_NAME);
    pivotEncoder = new CANcoder(17, Robot.CANIVORE_NAME);
    intakeMotor = new TalonFX(18, Robot.CANIVORE_NAME);

    final double pivotGearRatio = 100;
    final var pivotMotor = DCMotor.getKrakenX60Foc(1).withReduction(pivotGearRatio);

    var pivotConfig = new TalonFXConfiguration();
    pivotConfig.CurrentLimits.StatorCurrentLimit = 80;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = 60;
    pivotConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    pivotConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
    pivotConfig.Feedback.RotorToSensorRatio = pivotGearRatio;
    pivotConfig.Feedback.SensorToMechanismRatio = 1;
    pivotConfig.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    pivotConfig.Slot0.kV = 1 / Units.radiansToRotations(pivotMotor.KvRadPerSecPerVolt);
    pivotConfig.Audio.AllowMusicDurDisable = true;

    final var intakeMotor = DCMotor.getKrakenX60Foc(1);

    var intakeConfig = new TalonFXConfiguration();
    intakeConfig.CurrentLimits.StatorCurrentLimit = 60;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = 40;
    intakeConfig.CurrentLimits.SupplyCurrentLowerLimit = 10;
    intakeConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
    intakeConfig.Slot0.kV = 1 / Units.radiansToRotations(intakeMotor.KvRadPerSecPerVolt);
  }

  @Override
  public void updateInputs(AlgaeClawIOInputs inputs) {
    AlgaeClawIO.super.updateInputs(inputs);
  }

  @Override
  public void setPivotPos(double angleRad) {
    AlgaeClawIO.super.setPivotPos(angleRad);
  }

  @Override
  public void setPivotVel(double velRadPerSec) {
    AlgaeClawIO.super.setPivotVel(velRadPerSec);
  }

  @Override
  public void runIntake(double speedMetersPerSec) {
    AlgaeClawIO.super.runIntake(speedMetersPerSec);
  }
}
