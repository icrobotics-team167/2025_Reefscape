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
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.cotc.Robot;
import frc.cotc.util.PhoenixBatchRefresher;

public class ElevatorIOPhoenix implements ElevatorIO {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  private static final double gearRatio = 10;
  private static final double spoolDiameter = .1;
  private static final double metersPerRotation = spoolDiameter * Math.PI;

  private Sim sim;

  public ElevatorIOPhoenix() {
    leftMotor = new TalonFX(13, Robot.CANIVORE_NAME);
    rightMotor = new TalonFX(14, Robot.CANIVORE_NAME);

    posSignal = leftMotor.getPosition(false);
    velSignal = leftMotor.getVelocity(false);
    leftStator = leftMotor.getStatorCurrent(false);
    leftSupply = leftMotor.getSupplyCurrent(false);
    rightStator = rightMotor.getStatorCurrent(false);
    rightSupply = rightMotor.getSupplyCurrent(false);
    PhoenixBatchRefresher.register(
        posSignal, velSignal, leftStator, leftSupply, rightStator, rightSupply);
    ParentDevice.optimizeBusUtilizationForAll(4, leftMotor, rightMotor);

    var config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = gearRatio;
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;
    leftMotor.getConfigurator().apply(config);
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightMotor.getConfigurator().apply(config);

    var motorModel = DCMotor.getKrakenX60Foc(2).withReduction(gearRatio);

    if (Robot.isSimulation()) {
      sim =
          new Sim(
              leftMotor,
              rightMotor,
              12.0 / (Units.radiansToRotations(motorModel.freeSpeedRadPerSec) * metersPerRotation),
              0.01,
              motorModel);
    }
  }

  private final BaseStatusSignal posSignal;
  private final BaseStatusSignal velSignal;
  private final BaseStatusSignal leftStator;
  private final BaseStatusSignal leftSupply;
  private final BaseStatusSignal rightStator;
  private final BaseStatusSignal rightSupply;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (Robot.isSimulation()) {
      sim.run();
    }

    inputs.posMeters = posSignal.getValueAsDouble() * metersPerRotation;
    inputs.velMetersPerSec = velSignal.getValueAsDouble() * metersPerRotation;
    inputs.leftMotorCurrentDraws.mutateFromSignals(leftStator, leftSupply);
    inputs.rightMotorCurrentDraws.mutateFromSignals(rightStator, rightSupply);
  }

  private final VoltageOut voltageControl = new VoltageOut(0);

  @Override
  public void runVoltage(double volts) {
    leftMotor.setControl(voltageControl.withOutput(volts));
    rightMotor.setControl(voltageControl.withOutput(volts));
  }

  private static class Sim {
    private final TalonFXSimState leftMotorSim;
    private final TalonFXSimState rightMotorSim;

    private final ElevatorSim elevatorSim;

    Sim(TalonFX leftMotor, TalonFX rightMotor, double kV, double kA, DCMotor motorModel) {
      leftMotorSim = leftMotor.getSimState();
      rightMotorSim = rightMotor.getSimState();
      rightMotorSim.Orientation = ChassisReference.Clockwise_Positive;
      elevatorSim = new ElevatorSim(kV, kA, motorModel, 0, 2, true, 0);
    }

    private final double dt = Robot.defaultPeriodSecs;

    void run() {
      elevatorSim.setInputVoltage(leftMotorSim.getMotorVoltage());
      elevatorSim.update(dt);

      leftMotorSim.setRawRotorPosition(
          elevatorSim.getPositionMeters() / metersPerRotation * gearRatio);
      leftMotorSim.setRotorVelocity(
          elevatorSim.getVelocityMetersPerSecond() / metersPerRotation * gearRatio);
      rightMotorSim.setRawRotorPosition(
          elevatorSim.getPositionMeters() / metersPerRotation * gearRatio);
      rightMotorSim.setRotorVelocity(
          elevatorSim.getVelocityMetersPerSecond() / metersPerRotation * gearRatio);
    }
  }
}
