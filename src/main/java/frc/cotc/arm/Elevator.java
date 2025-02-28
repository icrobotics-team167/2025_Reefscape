// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.cotc.Robot;
import frc.cotc.util.GainsCalculator;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIO.ElevatorIOInputs inputs = new ElevatorIO.ElevatorIOInputs();

  private final double firstStageKg;
  private final GainsCalculator.PDGains firstStageGains;
  private final double secondStageKg;
  private final GainsCalculator.PDGains secondStageGains;

  private final double switchPoint;
  private final double maxHeight;

  private final LoggedMechanism2d visualization;
  private final LoggedMechanismLigament2d stage1Ligament;
  private final LoggedMechanismLigament2d stage2Ligament;

  public Elevator(ElevatorIO io) {
    this.io = io;

    var constants = io.getConstants();
    Logger.processInputs("Superstructure/Elevator/CONSTANTS", constants);
    firstStageKg = constants.kG_firstStage;
    firstStageGains =
        GainsCalculator.getPositionGains(
            constants.kV,
            constants.kG_firstStage / 9.81,
            12 - constants.kG_firstStage,
            .01,
            .075,
            Robot.defaultPeriodSecs,
            0.001);
    secondStageKg = constants.kG_secondStage;
    secondStageGains =
        GainsCalculator.getPositionGains(
            constants.kV,
            constants.kG_secondStage / 9.81,
            12 - constants.kG_secondStage,
            .01,
            .075,
            Robot.defaultPeriodSecs,
            0.001);

    switchPoint = constants.switchPointMeters;
    maxHeight = constants.maxHeightMeters;

    visualization = new LoggedMechanism2d(Units.inchesToMeters(28), 3);
    stage1Ligament = new LoggedMechanismLigament2d("stage1", 1, 90, 5, new Color8Bit(255, 0, 0));
    stage2Ligament = new LoggedMechanismLigament2d("stage2", 1, 90, 7.5, new Color8Bit(255, 0, 0));
    var root =
        visualization.getRoot("root", Units.inchesToMeters(28 - 6), Units.inchesToMeters(.5));
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
        .setPosition(Units.inchesToMeters(28 - 6), inputs.posMeters + Units.inchesToMeters(18.5));
    Logger.recordOutput("Superstructure/Elevator/Visualization", visualization);
  }

  public Command retract() {
    return goToPos(0);
  }

  public Command lvl1() {
    return goToPos(.3);
  }

  public Command lvl2() {
    return goToPos(.5);
  }

  public Command lvl3() {
    return goToPos(.9);
  }

  public Command lvl4() {
    return goToPos(1.5);
  }

  public Command manualControl(DoubleSupplier control) {
    return run(
        () ->
            io.runVoltage(
                control.getAsDouble()
                    + (inputs.posMeters <= switchPoint ? firstStageKg : secondStageKg)));
  }

  private double targetHeight = 0;

  public boolean atTargetPos() {
    return Math.abs(inputs.posMeters - targetHeight) < .025;
  }

  private final PIDController feedbackController = new PIDController(0, 0, 0);

  private Command goToPos(double posMeters) {
    return runOnce(
            () -> {
              feedbackController.reset();
              targetHeight = posMeters;
            })
        .andThen(
            run(
                () -> {
                  var gains = inputs.posMeters <= switchPoint ? firstStageGains : secondStageGains;
                  var ff = inputs.posMeters <= switchPoint ? firstStageKg : secondStageKg;

                  feedbackController.setPID(gains.kP(), 0, gains.kD());
                  var feedbackVoltage =
                      feedbackController.calculate(
                          inputs.posMeters, MathUtil.clamp(posMeters, 0, maxHeight));
                  if (inputs.posMeters < .25) {
                    feedbackVoltage *= MathUtil.interpolate(.25, 1, inputs.posMeters / .25);
                  }
                  Logger.recordOutput("Superstructure/Elevator/Feedback", feedbackVoltage);

                  if (posMeters == 0
                      && MathUtil.isNear(0, inputs.posMeters, .01)
                      && MathUtil.isNear(0, inputs.velMetersPerSec, .05)) {
                    io.brake();
                  } else {
                    io.runVoltage(MathUtil.clamp(feedbackVoltage + ff, -12, 12));
                  }
                }));
  }
}
