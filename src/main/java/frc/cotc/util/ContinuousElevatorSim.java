// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalIntegration;
import org.ejml.simple.SimpleMatrix;

public class ContinuousElevatorSim {
  private final LinearSystem<N2, N1, N2> m_plantFirst;

  private final LinearSystem<N2, N1, N2> m_plantSecond;

  private final double m_switchPoint;
  private final double m_maxHeight;

  private Matrix<N2, N1> m_x;
  private Matrix<N1, N1> m_u;
  private Matrix<N2, N1> m_y;

  public ContinuousElevatorSim(
      LinearSystem<N2, N1, N2> firstStage,
      LinearSystem<N2, N1, N2> secondStage,
      double switchPoint,
      double maxHeight) {
    m_plantFirst = firstStage;
    m_plantSecond = secondStage;

    m_switchPoint = switchPoint;
    m_maxHeight = maxHeight;

    m_x = new Matrix<>(new SimpleMatrix(firstStage.getA().getNumRows(), 1));
    m_u = new Matrix<>(new SimpleMatrix(secondStage.getB().getNumCols(), 1));
    m_y = new Matrix<>(new SimpleMatrix(firstStage.getC().getNumRows(), 1));
  }

  public void update(double dtSeconds) {
    var plant = m_y.get(0, 0) < m_switchPoint ? m_plantFirst : m_plantSecond;

    // Update X. We hotswap the system plant depending on if we're on first stage extension or
    // second stage extension
    // Calculate updated x-hat from Runge-Kutta.
    var updatedXhat =
        NumericalIntegration.rkdp(
            (x, _u) -> {
              Matrix<N2, N1> xdot = plant.getA().times(x).plus(plant.getB().times(_u));
              xdot = xdot.plus(VecBuilder.fill(0, -9.81));
              return xdot;
            },
          m_x,
          m_u,
            dtSeconds);

    // We check for collisions after updating x-hat.
    if (updatedXhat.get(0, 0) <= 0) {
      m_x = VecBuilder.fill(0, 0);
    } else if (updatedXhat.get(0, 0) >= m_maxHeight) {
      m_x = VecBuilder.fill(m_maxHeight, 0);
    } else {
      m_x = updatedXhat;
    }

    // y = cx + du
    m_y = plant.calculateY(m_x, m_u);
  }

  public void setInputVoltage(double volts) {
    m_u =
        StateSpaceUtil.desaturateInputVector(
            new Matrix<>(new SimpleMatrix(m_u.getNumRows(), 1, true, volts)), 12);
  }

  public double getPositionMeters() {
    return m_y.get(0, 0);
  }

  public double getVelocityMetersPerSecond() {
    return m_y.get(1, 0);
  }
}
