// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.cotc.arm.ElevatorIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class ElevatorSubsystem extends SubsystemBase {

        private final ElevatorIO io;
        private final ElevatorIO.ElevatorIOInputs inputs = new ElevatorIO.ElevatorIOInputs();

        private final double switchPoint;
        private final double maxHeight;

        private final double firstStageKg;
        private final GainsCalculator.PDGains firstStageGains;
        private final double secondStageKg;
        private final GainsCalculator.PDGains secondStageGains;

        private final PIDController feedbackController = new PIDController(0, 0, 0);
        private double targetHeight = 0;

        private final LoggedMechanism2d visualization;
        private final LoggedMechanismLigament2d stage1Ligament;
        private final LoggedMechanismLigament2d stage2Ligament;

        public ElevatorSubsystem(ElevatorIO io) {
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
        }

        public boolean atTargetPos() {
            return Math.abs(inputs.posMeters - targetHeight) < 0.025;
        }

        public Command reset() {
            return goToPos(0);
        }

        public Command goToLevel(double position) {
            return goToPos(position);
        }

        public Command manualControl(DoubleSupplier control) {
            return run(() -> io.runVoltage(control.getAsDouble() + calculateFeedForward()));
        }

        private double calculateFeedForward() {
            return inputs.posMeters <= switchPoint ? io.getConstants().kG_firstStage : io.getConstants().kG_secondStage;
        }

        private Command goToPos(double posMeters) {
            return runOnce(() -> {
                feedbackController.reset();
                targetHeight = posMeters;
            }).andThen(run(() -> {
                double clampedTarget = Math.max(0, Math.min(posMeters, maxHeight));
                double feedbackVoltage = feedbackController.calculate(inputs.posMeters, clampedTarget);

                if (inputs.posMeters < 0.25) {
                    feedbackVoltage *= Math.min(1, inputs.posMeters / 0.25);
                }

                double totalVoltage = feedbackVoltage + calculateFeedForward();

                if (posMeters == 0 && atTargetPos()) {
                    io.brake();
                } else {
                    io.runVoltage(Math.max(-12, Math.min(12, totalVoltage)));
                }
            }));
        }