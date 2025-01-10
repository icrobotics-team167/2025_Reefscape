// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class FOCArmSim {
  private final DCMotor motor;
  // kg * m^2
  private final double moi;
  // nM * cos(theta)
  private final double gravityTorque;

  private final double minAngleRad;
  private final double maxAngleRad;

  public FOCArmSim(
      DCMotor motor,
      double massKg,
      double CoMRadiusMeters,
      double minAngleRad,
      double maxAngleRad,
      double initAngleRad) {
    this.motor = motor;
    moi = SingleJointedArmSim.estimateMOI(CoMRadiusMeters, massKg);
    gravityTorque = massKg * 9.81 * CoMRadiusMeters;
    this.minAngleRad = minAngleRad;
    this.maxAngleRad = maxAngleRad;

    posRad = initAngleRad;
  }

  private double posRad;
  private double velRadPerSec = 0;
  private double accelRadPerSecSquared = 0;

  /**
   * Ticks the simulation one timestep forwards.
   *
   * @param current The torque current of the motor. Should be positive for forward accel and
   *     negative for backwards accel.
   * @param dt The delta time in which to tick forwards.
   */
  public void tick(double current, double dt) {
    double maxCurrentDraw;
    // If accelerating (aka current and vel have the same sign), then limit current based on
    // current vel
    if (Math.signum(current) == Math.signum(velRadPerSec)) {
      // Clamp current draw to the max possible draw based on current vel
      maxCurrentDraw =
          MathUtil.interpolate(
              motor.stallCurrentAmps,
              0,
              MathUtil.inverseInterpolate(0, motor.freeSpeedRadPerSec, Math.abs(velRadPerSec)));

    } else {
      // Otherwise, clamp based on max stall current
      maxCurrentDraw = motor.stallCurrentAmps;
    }
    current = MathUtil.clamp(current, -maxCurrentDraw, maxCurrentDraw);

    double motorTorque = motor.getTorque(current);
    double gravityTorque = this.gravityTorque * Math.cos(posRad);
    double netTorque = motorTorque - gravityTorque;

    if ((netTorque > 0 || velRadPerSec > 0) && posRad >= maxAngleRad) {
      netTorque = 0;
      posRad = maxAngleRad;
      velRadPerSec = 0;
    } else if ((netTorque < 0 || velRadPerSec < 0) && posRad <= minAngleRad) {
      netTorque = 0;
      posRad = minAngleRad;
      velRadPerSec = 0;
    }
    // Torque / moi = acceleration
    accelRadPerSecSquared = netTorque / moi;
    // pos = p_0 + v_0 * dt + (1/2 * a * dt^2)
    posRad += velRadPerSec * dt + (accelRadPerSecSquared * dt * dt / 2);
    velRadPerSec += accelRadPerSecSquared * dt;
  }

  /**
   * @return The position in radians.
   */
  public double getPosRad() {
    return posRad;
  }

  /**
   * @return The velocity in radians per second.
   */
  public double getVelRadPerSec() {
    return velRadPerSec;
  }

  /**
   * @return The velocity in radians per second squared.
   */
  public double getAccelRadPerSecSquared() {
    return accelRadPerSecSquared;
  }
}
