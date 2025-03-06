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
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.cotc.Robot;
import frc.cotc.util.PhoenixBatchRefresher;

public class AlgaePivotIOPhoenix implements AlgaePivotIO {
  private final TalonFX motor;

  private final BaseStatusSignal posSignal, velSignal, statorSignal, supplySignal;

  public AlgaePivotIOPhoenix() {
    motor = new TalonFX(3);

    var config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimit = 20;
    config.Feedback.SensorToMechanismRatio = (32.0 / 18.0) * 81;
    config.Slot0.kV = 12.0 / ((7530.0 / 60.0) / config.Feedback.SensorToMechanismRatio);
    config.Slot0.kG = .32;
    config.Slot0.kA = config.Slot0.kG / 9.81 * 2 * Math.PI * Units.inchesToMeters(12);
    config.Slot0.kP = 96;
    config.Slot0.kD = 1;
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    motor.getConfigurator().apply(config);

    posSignal = motor.getPosition(false);
    velSignal = motor.getVelocity(false);
    statorSignal = motor.getStatorCurrent(false);
    supplySignal = motor.getSupplyCurrent(false);
    BaseStatusSignal.setUpdateFrequencyForAll(50, posSignal, velSignal, statorSignal, supplySignal);
    motor.optimizeBusUtilization(5, .01);
    PhoenixBatchRefresher.registerRio(posSignal, velSignal, statorSignal, supplySignal);

    if (Robot.isSimulation()) {
      new Sim(motor, config.Slot0.kV, config.Slot0.kA, config.Feedback.SensorToMechanismRatio)
          .start();
    }

    motor.setPosition(Units.degreesToRotations(-75));
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

  @Override
  public void manualOverride(double volts) {
    motor.setVoltage(volts);
  }

  private static class Sim {
    private final TalonFXSimState motorSim;
    private final SingleJointedArmSim armSim;

    private final double gearRatio;

    Sim(TalonFX motor, double kv, double ka, double gearRatio) {
      motorSim = motor.getSimState();
      motorSim.Orientation = ChassisReference.Clockwise_Positive;

      armSim =
          new SingleJointedArmSim(
              LinearSystemId.identifyPositionSystem(kv / (2 * Math.PI), ka / (2 * Math.PI)),
              new DCMotor(12, 4.05, 275, 1.4, Units.rotationsPerMinuteToRadiansPerSecond(7530), 1),
              gearRatio,
              Units.inchesToMeters(18),
              Units.degreesToRadians(-75),
              Units.degreesToRadians(120),
              true,
              Units.degreesToRadians(-75));
      this.gearRatio = gearRatio;
    }

    private final double dt = 1.0 / 250;

    private final Notifier notifier = new Notifier(this::run);

    void start() {
      notifier.startPeriodic(dt);
    }

    private void run() {
      armSim.setInputVoltage(motorSim.getMotorVoltage());
      armSim.update(dt);

      motorSim.setRawRotorPosition(Units.radiansToRotations(armSim.getAngleRads()) * gearRatio);
      motorSim.setRotorVelocity(
          Units.radiansToRotations(armSim.getVelocityRadPerSec()) * gearRatio);
    }
  }
}
