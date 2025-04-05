// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import frc.cotc.Robot;
import frc.cotc.util.ReefLocations;

public class AlgaeRollersIOSim implements AlgaeRollersIO {
  private boolean hasAlgae = false;

  @Override
  public void updateInputs(AlgaeRollersIOInputs inputs) {
    inputs.hasAlgae = hasAlgae;
  }

  @Override
  public void intake() {
    if (Robot.groundTruthPoseSupplier == null) {
      return;
    }

    var reef = Robot.isOnRed() ? ReefLocations.RED_REEF : ReefLocations.BLUE_REEF;

    var groundTruthPose = Robot.groundTruthPoseSupplier.get();
    var botToReef = reef.minus(groundTruthPose.getTranslation());

    if (!hasAlgae) {
      hasAlgae = botToReef.getNorm() < 1.4;
    }
  }

  @Override
  public void eject() {
    hasAlgae = false;
  }
}
