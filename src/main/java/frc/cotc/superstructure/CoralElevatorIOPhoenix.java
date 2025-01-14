// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.cotc.Robot;
import frc.cotc.util.PhoenixBatchRefresher;

public class CoralElevatorIOPhoenix implements CoralElevatorIO {
  private final TalonFX motor;

  private final double gearRatio = 100;
  private final DCMotor motorModel = DCMotor.getKrakenX60Foc(1).withReduction(gearRatio);

  private final double metersPerRotation = Units.inchesToMeters(4) * Math.PI;

  private final BaseStatusSignal posSignal, velSignal, statorSignal, supplySignal;

  public CoralElevatorIOPhoenix() {
    motor = new TalonFX(16, Robot.CANIVORE_NAME);

    var config = new TalonFXConfiguration();
    config.Audio.AllowMusicDurDisable = true;
    config.MotionMagic.MotionMagicExpo_kV =
        12 / Units.radiansToRotations(motorModel.freeSpeedRadPerSec);
    config.MotionMagic.MotionMagicAcceleration = .1;
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Feedback.SensorToMechanismRatio = gearRatio;
    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLimit = 40;

    if (Robot.isReal()) {
      config.MotionMagic.MotionMagicExpo_kA = 0.1;
    } else {
      config.MotionMagic.MotionMagicExpo_kA = .025;
      config.Slot0.kG = 0.625;
      config.Slot0.kA = 1;
    }

    motor.getConfigurator().apply(config);

    posSignal = motor.getPosition(false);
    velSignal = motor.getVelocity(false);
    statorSignal = motor.getStatorCurrent(false);
    supplySignal = motor.getSupplyCurrent(false);

    BaseStatusSignal.setUpdateFrequencyForAll(250, posSignal, velSignal);
    BaseStatusSignal.setUpdateFrequencyForAll(50, statorSignal, supplySignal);

    motor.optimizeBusUtilization(4, .1);

    PhoenixBatchRefresher.register(posSignal, velSignal, statorSignal, supplySignal);

    if (Robot.isSimulation()) {
      initSim();
      simNotifier.startPeriodic(simDt);
    }
  }

  @Override
  public void updateInputs(CoralElevatorIOInputs inputs) {
    inputs.posMeters = posSignal.getValueAsDouble() * metersPerRotation;
    inputs.velMetersPerSec = velSignal.getValueAsDouble() * metersPerRotation;

    inputs.currentDraws.mutateFromSignals(statorSignal, supplySignal);
  }

  private final MotionMagicTorqueCurrentFOC positionControl = new MotionMagicTorqueCurrentFOC(0);

  @Override
  public void goToPos(double posMeters) {
    motor.setControl(positionControl.withPosition(posMeters / metersPerRotation));
  }

  private final TorqueCurrentFOC characterizationControl = new TorqueCurrentFOC(0);

  @Override
  public void characterize(double output) {
    motor.setControl(characterizationControl.withOutput(output));
  }

  private final double simDt = .001;
  private Notifier simNotifier;
  private TalonFXSimState motorSim;
  private ElevatorSim elevatorSim;

  private void initSim() {
    motorSim = motor.getSimState();
    elevatorSim =
        new ElevatorSim(
            motorModel,
            1,
            Units.lbsToKilograms(5),
            metersPerRotation / (2 * Math.PI),
            0,
            5,
            true,
            0);

    simNotifier = new Notifier(this::tickSim);
  }

  private void tickSim() {
    elevatorSim.setInputVoltage(motorSim.getMotorVoltage());
    elevatorSim.update(simDt);

    motorSim.setRawRotorPosition(elevatorSim.getPositionMeters() / metersPerRotation * gearRatio);
    motorSim.setRotorVelocity(
        elevatorSim.getVelocityMetersPerSecond() / metersPerRotation * gearRatio);
  }
}
