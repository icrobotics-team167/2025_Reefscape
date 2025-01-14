// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.cotc.Robot;
import frc.cotc.util.PhoenixBatchRefresher;

public class CoralElevatorIOPhoenix implements CoralElevatorIO {
  private final TalonFX motor;

  private final double gearRatio = 50;
  private final DCMotor motorModel = DCMotor.getKrakenX60Foc(1).withReduction(gearRatio);

  private final double metersPerRotation = Units.inchesToMeters(4) * Math.PI;

  private final BaseStatusSignal posSignal, velSignal, statorSignal, supplySignal;

  public CoralElevatorIOPhoenix() {
    motor = new TalonFX(16, Robot.CANIVORE_NAME);

    var config = new TalonFXConfiguration();
    config.Audio.AllowMusicDurDisable = true;
    config.Slot0.kV = 12 / Units.radiansToRotations(motorModel.freeSpeedRadPerSec);
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Feedback.SensorToMechanismRatio = gearRatio;
    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.SupplyCurrentLimit = 40;

    if (Robot.isReal()) {
      config.Slot0.kG = 0;
    } else {
      config.Slot0.kG = 0.063845;
      config.Slot0.kA = 0.0042829;
      config.Slot0.kP = 80;
      config.Slot0.kD = 1.5;
    }

    config.MotionMagic.MotionMagicExpo_kV = config.Slot0.kV;
    config.MotionMagic.MotionMagicExpo_kA = config.Slot0.kA;

    motor.getConfigurator().apply(config);

    posSignal = motor.getPosition(false);
    velSignal = motor.getVelocity(false);
    statorSignal = motor.getStatorCurrent(false);
    supplySignal = motor.getSupplyCurrent(false);

    BaseStatusSignal.setUpdateFrequencyForAll(50, posSignal, velSignal, statorSignal, supplySignal);

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

  private final MotionMagicExpoVoltage positionControl = new MotionMagicExpoVoltage(0);
  private final StaticBrake brakeControl = new StaticBrake();

  @Override
  public void goToPos(double posMeters) {
    if (MathUtil.isNear(0, posMeters, .01)
        && MathUtil.isNear(0, posSignal.getValueAsDouble(), .01)) {
      motor.setControl(brakeControl);
    } else {
      motor.setControl(positionControl.withPosition(posMeters / metersPerRotation));
    }
  }

  private final VoltageOut characterizationControl = new VoltageOut(0);

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
            Units.lbsToKilograms(10),
            metersPerRotation / (2 * Math.PI),
            0,
            2.2,
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
