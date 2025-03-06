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
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import frc.cotc.Robot;
import frc.cotc.util.ContinuousElevatorSim;
import frc.cotc.util.GainsCalculator;
import frc.cotc.util.PhoenixBatchRefresher;

public class ElevatorIOPhoenix implements ElevatorIO {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final DigitalInput limitSwitch;

  private static final double gearRatio;
  private static final double metersPerRotation;

  private static final ElevatorIOConstantsAutoLogged constants;
  private static final double maxHeightMeters = 1.53551952554;

  static {
    gearRatio = (50.0 / 12.0) * (52.0 / 20.0);

    var pitch = 5.0 / 1000;
    var teeth = 36;
    metersPerRotation = teeth * pitch;

    constants = new ElevatorIOConstantsAutoLogged();
    constants.switchPointMeters = 0.76981640676;
  }

  public ElevatorIOPhoenix() {
    leftMotor = new TalonFX(13, Robot.CANIVORE_NAME);
    rightMotor = new TalonFX(14, Robot.CANIVORE_NAME);
    limitSwitch = new DigitalInput(3);

    posSignal = leftMotor.getPosition(false);
    velSignal = leftMotor.getVelocity(false);
    leftStator = leftMotor.getStatorCurrent(false);
    leftSupply = leftMotor.getSupplyCurrent(false);
    rightStator = rightMotor.getStatorCurrent(false);
    rightSupply = rightMotor.getSupplyCurrent(false);
    PhoenixBatchRefresher.registerCanivore(
        posSignal, velSignal, leftStator, leftSupply, rightStator, rightSupply);
    BaseStatusSignal.setUpdateFrequencyForAll(100, posSignal, velSignal);
    BaseStatusSignal.setUpdateFrequencyForAll(50, leftStator, leftSupply, rightStator, rightSupply);
    ParentDevice.optimizeBusUtilizationForAll(5, leftMotor, rightMotor);

    var config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = gearRatio;
    config.CurrentLimits.StatorCurrentLimit = 50;
    config.CurrentLimits.SupplyCurrentLimit = 50;
    config.CurrentLimits.SupplyCurrentLowerLimit = 10;
    config.CurrentLimits.SupplyCurrentLowerTime = 1.5;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxHeightMeters / metersPerRotation;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.kV = 12 / ((5800.0 / 60.0) / gearRatio);
    config.Slot0.kG = .18;
    config.Slot0.kA = (config.Slot0.kG / 9.81) * metersPerRotation;
    var slot0gains =
        GainsCalculator.getPositionGains(
            config.Slot0.kV, config.Slot0.kA, 12 - config.Slot0.kG, .01, .075, .001, .001);
    config.Slot0.kP = slot0gains.kP();
    config.Slot0.kD = slot0gains.kD();
    config.Slot1.kV = config.Slot0.kV;
    config.Slot1.kG = .192;
    config.Slot1.kA = (config.Slot1.kG / 9.81) * metersPerRotation;
    var slot1gains =
        GainsCalculator.getPositionGains(
            config.Slot1.kV, config.Slot1.kA, 12 - config.Slot0.kG, .01, .075, .001, .001);
    config.Slot1.kP = slot1gains.kP();
    config.Slot1.kD = slot1gains.kD();

    leftMotor.getConfigurator().apply(config);
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightMotor.getConfigurator().apply(config);

    if (Robot.isSimulation()) {
      new Sim(
              leftMotor,
              rightMotor,
              config.Slot0.kV / metersPerRotation,
              config.Slot0.kA / metersPerRotation,
              config.Slot1.kA / metersPerRotation)
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

  private boolean lastTriggeredState = false;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    var triggeredState = !limitSwitch.get(); // Normally closed
    if (!lastTriggeredState && triggeredState) {
      leftMotor.setPosition(0);
    }
    lastTriggeredState = triggeredState;

    inputs.posMeters =
        triggeredState
            ? 0
            : BaseStatusSignal.getLatencyCompensatedValueAsDouble(posSignal, velSignal)
                * metersPerRotation;
    inputs.velMetersPerSec = velSignal.getValueAsDouble() * metersPerRotation;
    inputs.leftMotorCurrentDraws.mutateFromSignals(leftStator, leftSupply);
    inputs.rightMotorCurrentDraws.mutateFromSignals(rightStator, rightSupply);
  }

  private final PositionVoltage positionControl = new PositionVoltage(0);

  @Override
  public void goToPos(double posMeters) {
    posMeters = MathUtil.clamp(posMeters, 0, maxHeightMeters);
    if (posSignal.getValueAsDouble() * metersPerRotation > constants.switchPointMeters) {
      positionControl.Slot = 1;
    } else {
      positionControl.Slot = 0;
    }
    positionControl.Position = posMeters / metersPerRotation;
    leftMotor.setControl(positionControl);
    rightMotor.setControl(positionControl);
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
      leftMotorSim.Orientation = ChassisReference.Clockwise_Positive;
      elevatorSim =
          new ContinuousElevatorSim(
              LinearSystemId.identifyPositionSystem(kV, firstStage_kA),
              LinearSystemId.identifyPositionSystem(kV, secondStage_kA),
              constants.switchPointMeters,
              maxHeightMeters);
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
