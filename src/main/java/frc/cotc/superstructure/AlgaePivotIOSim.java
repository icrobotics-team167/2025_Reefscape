// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.cotc.Robot;

public class AlgaePivotIOSim implements AlgaePivotIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          new DCMotor(
              12,
              4.05 * (9.37 / 7.09),
              275 * (483.0 / 366.0),
              1.4,
              Units.rotationsPerMinuteToRadiansPerSecond(7530 * (5800.0 / 6000.0)),
              1),
          (46.0 / 42.0) * 81,
          SingleJointedArmSim.estimateMOI(Units.inchesToMeters(12), Units.lbsToKilograms(10)),
          Units.inchesToMeters(12),
          Units.degreesToRadians(-75),
          Units.degreesToRadians(120),
          true,
          Units.degreesToRadians(-75));

  private double targetPos = Units.degreesToRadians(-75);

  private final PIDController pid = new PIDController(48, 0, 1);

  @Override
  public void updateInputs(AlgaePivotIOInputs inputs) {
    sim.setInputVoltage(
        pid.calculate(sim.getAngleRads(), targetPos) + .1 * Math.cos(sim.getAngleRads()));
    sim.update(Robot.defaultPeriodSecs);

    inputs.posRad = sim.getAngleRads();
    inputs.velRadPerSec = sim.getVelocityRadPerSec();
  }

  @Override
  public void intake() {
    targetPos = Units.degreesToRadians(-35);
  }

  @Override
  public void stow() {
    targetPos = Units.degreesToRadians(-75);
  }

  @Override
  public void hold() {
    targetPos = Units.degreesToRadians(120);
  }
}
