// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.math.filter.Debouncer;
import frc.cotc.Robot;
import frc.cotc.util.ReefLocations;
import java.util.function.BooleanSupplier;

public class AlgaeRollersIOSim implements AlgaeRollersIO {
  private boolean hasAlgae = false;

  @Override
  public void updateInputs(AlgaeRollersIOInputs inputs) {
    inputs.hasAlgae = hasAlgae;
  }

  BooleanSupplier atTargetAngle;
  BooleanSupplier atTargetHeight;

  Debouncer debouncer = new Debouncer(.2);

  @Override
  public void intake() {
    if (Robot.groundTruthPoseSupplier == null) {
      return;
    }

    var reef = Robot.isOnRed() ? ReefLocations.RED_REEF : ReefLocations.BLUE_REEF;

    var groundTruthPose = Robot.groundTruthPoseSupplier.get();
    var groundTruthSpeed = Robot.groundTruthSpeedSupplier.get();
    var botToReef = reef.minus(groundTruthPose.getTranslation());

    if (!hasAlgae) {
      hasAlgae =
          debouncer.calculate(
              botToReef.getNorm() < 1.4
                  && Math.hypot(
                          groundTruthSpeed.vxMetersPerSecond, groundTruthSpeed.vyMetersPerSecond)
                      < .1
                  && atTargetAngle.getAsBoolean()
                  && atTargetHeight.getAsBoolean());
    }
  }

  @Override
  public void eject() {
    hasAlgae = false;
  }
}
