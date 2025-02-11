// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import frc.cotc.Robot;
import frc.cotc.util.ContinuousElevatorSim;
import frc.cotc.util.PhoenixBatchRefresher;

public class ElevatorIOPhoenix implements ElevatorIO {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  private static final double gearRatio = 10;
  private static final double spoolDiameter = Units.inchesToMeters(2.256);
  private static final double metersPerRotation = spoolDiameter * Math.PI;

  private static final ElevatorIOConstantsAutoLogged constants;

  static {
    constants = new ElevatorIOConstantsAutoLogged();
    constants.kV = 12.0 / ((5800.0 / 60.0) / gearRatio * metersPerRotation);
    constants.kA_firstStage = .01;
    constants.kA_secondStage = .02;
    constants.switchPointMeters = 1;
    constants.maxHeightMeters = 2;
  }

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
    BaseStatusSignal.setUpdateFrequencyForAll(100, posSignal, velSignal);
    BaseStatusSignal.setUpdateFrequencyForAll(50, leftStator, leftSupply, rightStator, rightSupply);
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

    if (Robot.isSimulation()) {
      new Sim(
              leftMotor,
              rightMotor,
              constants.kV,
              constants.kA_firstStage,
              constants.kA_secondStage)
          .start();
    }
  }

  @Override
  public ElevatorIOConstantsAutoLogged getConstants() {
    return constants;
  }

  private final BaseStatusSignal posSignal;
  private final BaseStatusSignal velSignal;
  private final BaseStatusSignal leftStator;
  private final BaseStatusSignal leftSupply;
  private final BaseStatusSignal rightStator;
  private final BaseStatusSignal rightSupply;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.posMeters =
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(posSignal, velSignal)
            * metersPerRotation;
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

  private final StaticBrake brakeControl = new StaticBrake();

  @Override
  public void brake() {
    leftMotor.setControl(brakeControl);
    rightMotor.setControl(brakeControl);
  }

  private static class Sim {
    private final TalonFXSimState leftMotorSim;
    private final TalonFXSimState rightMotorSim;

    private final ContinuousElevatorSim elevatorSim;

    Sim(
        TalonFX leftMotor,
        TalonFX rightMotor,
        double kV,
        double firstStage_kA,
        double secondStage_kA) {
      leftMotorSim = leftMotor.getSimState();
      rightMotorSim = rightMotor.getSimState();
      rightMotorSim.Orientation = ChassisReference.Clockwise_Positive;
      elevatorSim =
          new ContinuousElevatorSim(
              LinearSystemId.identifyPositionSystem(kV, firstStage_kA),
              LinearSystemId.identifyPositionSystem(kV, secondStage_kA),
              constants.switchPointMeters,
              constants.maxHeightMeters);
    }

    private final double dt = 1.0 / 250;

    private final Notifier notifier = new Notifier(this::run);

    public void start() {
      notifier.startPeriodic(dt);
    }

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
