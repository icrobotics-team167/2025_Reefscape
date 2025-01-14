// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.system.LinearSystem;

/**
 * A calculator for generating optimal PD gains given known control characteristics.
 *
 * <p>Implementation is taken from SysID.
 */
public class GainsCalculator {
  public record PDGains(double kP, double kD) {}

  public static PDGains getPositionGains(
      double kV,
      double kA,
      double maxControlEffort,
      double positionTolerance,
      double velocityTolerance,
      double controllerPeriodSeconds,
      double measurementDelaySeconds) {
    if (!Double.isFinite(kV) || !Double.isFinite(kA)) {
      throw new IllegalArgumentException("kV and kA must be finite numbers!");
    }
    if (kV < 0) {
      throw new IllegalArgumentException("kV must be greater than or equal to zero!");
    }
    if (kA < 0) {
      throw new IllegalArgumentException("kA must be greater than or equal to zero!");
    }

    if (kA >= 1e-7) {
      var system =
          new LinearSystem<>(
              new Matrix<>(Nat.N2(), Nat.N2(), new double[] {0.0, 1.0, 0.0, -kV / kA}),
              new Matrix<>(Nat.N2(), Nat.N1(), new double[] {0.0, 1.0 / kA}),
              new Matrix<>(Nat.N1(), Nat.N2(), new double[] {1.0, 0.0}),
              new Matrix<>(Nat.N1(), Nat.N1(), new double[] {0.0}));
      var controller =
          new LinearQuadraticRegulator<>(
              system,
              VecBuilder.fill(positionTolerance, velocityTolerance),
              VecBuilder.fill(maxControlEffort),
              controllerPeriodSeconds);
      controller.latencyCompensate(system, controllerPeriodSeconds, measurementDelaySeconds);

      return new PDGains(controller.getK().get(0, 0), controller.getK().get(0, 1));
    }

    // Special case: kA ~= 0, in which case we change the math to avoid a divide-by-zero error
    // due to acceleration happening instantly.
    var system =
        new LinearSystem<>(
            new Matrix<>(Nat.N1(), Nat.N1(), new double[] {0.0}),
            new Matrix<>(Nat.N1(), Nat.N1(), new double[] {1.0}),
            new Matrix<>(Nat.N1(), Nat.N1(), new double[] {1.0}),
            new Matrix<>(Nat.N1(), Nat.N1(), new double[] {0.0}));
    var controller =
        new LinearQuadraticRegulator<>(
            system,
            VecBuilder.fill(positionTolerance),
            VecBuilder.fill(maxControlEffort),
            controllerPeriodSeconds);
    controller.latencyCompensate(system, controllerPeriodSeconds, measurementDelaySeconds);

    return new PDGains(kV * controller.getK().get(0, 0), 0);
  }

  public static double getVelocityPGain(
      double kV,
      double kA,
      double maxControlEffort,
      double tolerance,
      double controllerPeriodSeconds,
      double measurementDelaySeconds) {
    if (!Double.isFinite(kV) || !Double.isFinite(kA)) {
      throw new IllegalArgumentException("kV and kA must be finite numbers!");
    }
    if (kV < 0) {
      throw new IllegalArgumentException("kV must be greater than or equal to 0!");
    }
    if (kA < 0) {
      throw new IllegalArgumentException("kA must be greater than or equal to 0!");
    }

    // Special case: kA ~= 0, in which case there's no need for a P gain since acceleration
    // happens instantly.
    if (kA <= 1e-7) {
      return 0;
    }

    var system =
        new LinearSystem<>(
            new Matrix<>(Nat.N1(), Nat.N1(), new double[] {-kV / kA}),
            new Matrix<>(Nat.N1(), Nat.N1(), new double[] {1.0 / kA}),
            new Matrix<>(Nat.N1(), Nat.N1(), new double[] {1.0}),
            new Matrix<>(Nat.N1(), Nat.N1(), new double[] {0.0}));
    var controller =
        new LinearQuadraticRegulator<>(
            system,
            VecBuilder.fill(tolerance),
            VecBuilder.fill(maxControlEffort),
            controllerPeriodSeconds);
    controller.latencyCompensate(system, controllerPeriodSeconds, measurementDelaySeconds);

    return controller.getK().get(0, 0);
  }
}
