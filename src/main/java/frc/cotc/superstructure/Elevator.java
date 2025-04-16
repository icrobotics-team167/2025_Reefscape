// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final double switchPoint;
  private final double maxHeight;

  private final LoggedMechanism2d visualization;
  private final LoggedMechanismLigament2d stage1Ligament;
  private final LoggedMechanismLigament2d stage2Ligament;

  Elevator(ElevatorIO io) {
    this.io = io;

    var constants = io.getConstants();
    Logger.processInputs("Superstructure/Elevator/CONSTANTS", constants);

    switchPoint = constants.switchPointMeters;
    maxHeight = constants.maxHeightMeters;

    visualization = new LoggedMechanism2d(Units.inchesToMeters(28), 3);
    stage1Ligament = new LoggedMechanismLigament2d("stage1", 1, 90, 5, new Color8Bit(255, 0, 0));
    stage2Ligament = new LoggedMechanismLigament2d("stage2", 1, 90, 7.5, new Color8Bit(255, 0, 0));
    var root =
        visualization.getRoot("root", Units.inchesToMeters(28 - 7), Units.inchesToMeters(.5));
    root.append(stage1Ligament);
    root.append(stage2Ligament);
    root.append(
        new LoggedMechanismLigament2d(
            "base", Units.inchesToMeters(39), 90, 10, new Color8Bit(255, 0, 0)));
    visualization
        .getRoot("coralBase", 0, 0)
        .append(
            new LoggedMechanismLigament2d(
                "coral", Units.inchesToMeters(6), -35, 2.5, new Color8Bit(Color.kOrangeRed)));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure/Elevator", inputs);
    stage1Ligament.setLength(Units.inchesToMeters(40.5) + inputs.posMeters);
    stage2Ligament.setLength(
        inputs.posMeters < switchPoint
            ? Units.inchesToMeters(40)
            : Units.inchesToMeters(40) + inputs.posMeters - switchPoint);
    visualization
        .getRoot("coralBase", 0, 0)
        .setPosition(Units.inchesToMeters(28 - 6), inputs.posMeters + Units.inchesToMeters(20.5));
    Logger.recordOutput("Superstructure/Elevator/Visualization", visualization);
    Logger.recordOutput("Superstructure/Elevator/atTargetPos", atTargetPos());
  }

  Command retract() {
    return goToPos(0).withName("Retract");
  }

  Command lvl1() {
    return goToPos(.325).withName("Lvl 2");
  }

  Command lvl2() {
    return goToPos(.5).withName("Lvl 2");
  }

  Command lvl3() {
    return goToPos(0.9).withName("Lvl 3");
  }

  Command lvl4() {
    return goToPos(1.515).withName("Lvl 4");
  }

  Command lowAlgae() {
    return goToPos(.3).withName("Low Algae");
  }

  Command highAlgae() {
    return goToPos(.58).withName("High Algae");
  }

  Command net() {
    return goToPos(1.1).withName("Net");
  }

  private double targetHeight = 0;

  boolean atTargetPos() {
    return Math.abs(inputs.posMeters - targetHeight) < .025
        && Math.abs(inputs.velMetersPerSec) < .25;
  }

  @AutoLogOutput(key = "Superstructure/Elevator/Extension")
  double getExtension() {
    return inputs.posMeters / maxHeight;
  }

  private Command goToPos(double posMeters) {
    final double clampedPosMeters = MathUtil.clamp(posMeters, 0, maxHeight);
    return runOnce(
            () -> {
              targetHeight = clampedPosMeters;
              Logger.recordOutput("Superstructure/Elevator/Target height", targetHeight);
            })
        .andThen(
            run(
                () -> {
                  if (clampedPosMeters == 0
                      && inputs.posMeters < .02
                      && inputs.velMetersPerSec < .02) {
                    io.brake();
                  } else {
                    io.setTargetPos(clampedPosMeters);
                  }
                }));
  }
}
