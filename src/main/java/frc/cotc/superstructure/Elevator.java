// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.cotc.Robot;
import frc.cotc.util.GainsCalculator;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIO.ElevatorIOInputs inputs = new ElevatorIO.ElevatorIOInputs();

  private final ElevatorFeedforward firstStageFeedforward;
  private final GainsCalculator.PDGains firstStageGains;
  private final ElevatorFeedforward secondStageFeedforward;
  private final GainsCalculator.PDGains secondStageGains;

  private final double switchPoint;
  private final double maxHeight;

  public Elevator(ElevatorIO io) {
    this.io = io;

    var constants = io.getConstants();
    Logger.processInputs("Superstructure/Elevator/CONSTANTS", constants);
    firstStageFeedforward =
        new ElevatorFeedforward(constants.kS_firstStage, constants.kA_firstStage * 9.81, 0, 0);
    firstStageGains =
        GainsCalculator.getPositionGains(
            constants.kV,
            constants.kA_firstStage,
            12 - constants.kS_firstStage - (constants.kA_firstStage * 9.81),
            .01,
            .2,
            Robot.defaultPeriodSecs,
            0.001);
    secondStageFeedforward =
        new ElevatorFeedforward(constants.kS_secondStage, constants.kA_secondStage * 9.81, 0, 0);
    secondStageGains =
        GainsCalculator.getPositionGains(
            constants.kV,
            constants.kA_secondStage,
            12 - constants.kS_secondStage - (constants.kA_secondStage * 9.81),
            .01,
            .2,
            Robot.defaultPeriodSecs,
            0.001);

    switchPoint = constants.switchPointMeters;
    maxHeight = constants.maxHeightMeters;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure/Elevator", inputs);
  }

  public Command retract() {
    return goToPos(0);
  }

  public Command lvl1() {
    return goToPos(.5);
  }

  public Command lvl2() {
    return goToPos(1);
  }

  public Command lvl3() {
    return goToPos(1.5);
  }

  public Command lvl4() {
    return goToPos(1.5);
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
                  var ff =
                      inputs.posMeters <= switchPoint
                          ? firstStageFeedforward
                          : secondStageFeedforward;

                  feedbackController.setPID(gains.kP(), 0, gains.kD());
                  var feedbackVoltage =
                      feedbackController.calculate(
                          inputs.posMeters, MathUtil.clamp(posMeters, 0, maxHeight));
                  var feedfowardVoltage = ff.calculate(feedbackVoltage);
                  Logger.recordOutput("Superstructure/Elevator/Feedback", feedbackVoltage);

                  if (posMeters == 0
                      && MathUtil.isNear(0, inputs.posMeters, .01)
                      && MathUtil.isNear(0, inputs.velMetersPerSec, .01)) {
                    io.brake();
                  } else {
                    io.runVoltage(feedbackVoltage + feedfowardVoltage);
                  }
                }));
  }
}
