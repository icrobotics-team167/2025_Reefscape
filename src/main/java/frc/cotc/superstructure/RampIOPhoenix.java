// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.cotc.Robot;
import frc.cotc.util.PhoenixBatchRefresher;
import org.littletonrobotics.junction.Logger;

public class RampIOPhoenix implements RampIO {
  private final TalonFX motor;

  private final BaseStatusSignal velocitySignal;
  private final BaseStatusSignal statorSignal;
  private final BaseStatusSignal supplySignal;

  private final double gearReduction = 400;

  public RampIOPhoenix() {
    motor = new TalonFX(15, Robot.CANIVORE_NAME);

    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 10;
    config.CurrentLimits.SupplyCurrentLimit = 5;
    config.CurrentLimits.SupplyCurrentLowerLimit = 1;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motor.getConfigurator().apply(config);

    velocitySignal = motor.getVelocity(false);
    statorSignal = motor.getStatorCurrent(false);
    supplySignal = motor.getSupplyCurrent(false);

    if (Robot.isSimulation()) {
      simState = motor.getSimState();
      simState.Orientation = ChassisReference.Clockwise_Positive;
      armSim =
          new SingleJointedArmSim(
              new DCMotor(12, 4.05, 275, 1.4, Units.rotationsPerMinuteToRadiansPerSecond(7530), 1),
              gearReduction,
              .01,
              .5,
              Units.degreesToRadians(45),
              Units.degreesToRadians(80),
              true,
              Units.degreesToRadians(60));
    }

    BaseStatusSignal.setUpdateFrequencyForAll(50, velocitySignal, statorSignal, supplySignal);
    motor.optimizeBusUtilization(5, .01);
    PhoenixBatchRefresher.registerCanivore(velocitySignal, statorSignal, supplySignal);
  }

  @Override
  public void updateInputs(RampIOInputs inputs) {
    if (Robot.isSimulation()) {
      updateSim();
    }

    inputs.velRPM = velocitySignal.getValueAsDouble() * 60;
    inputs.currentDraws.mutateFromSignals(statorSignal, supplySignal);
  }

  private final VoltageOut moveControl = new VoltageOut(0);

  @Override
  public void raise() {
    motor.setControl(moveControl.withOutput(2));
  }

  @Override
  public void lower() {
    motor.setControl(moveControl.withOutput(-1));
  }

  @Override
  public void brake() {
    motor.stopMotor();
  }

  private TalonFXSimState simState;
  private SingleJointedArmSim armSim;

  private void updateSim() {
    armSim.setInputVoltage(simState.getMotorVoltage());
    armSim.update(Robot.defaultPeriodSecs);
    simState.setRawRotorPosition(Units.radiansToRotations(armSim.getAngleRads()) * gearReduction);
    simState.setRotorVelocity(
        Units.radiansToRotations(armSim.getVelocityRadPerSec()) * gearReduction);
    Logger.recordOutput(
        "Superstructure/Ramp (Sim)/Pos", Units.radiansToDegrees(armSim.getAngleRads()));
  }
}
