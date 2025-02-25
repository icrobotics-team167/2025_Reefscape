// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.cotc.drive.Swerve;
import frc.cotc.drive.SwerveIO;
import frc.cotc.drive.SwerveIOPhoenix;
import frc.cotc.superstructure.*;
import frc.cotc.util.CommandXboxControllerWithRumble;
import frc.cotc.util.PhoenixBatchRefresher;
import frc.cotc.util.ReefLocations;
import frc.cotc.vision.FiducialPoseEstimator;
import frc.cotc.vision.FiducialPoseEstimatorIO;
import frc.cotc.vision.FiducialPoseEstimatorIOPhoton;
import java.util.NoSuchElementException;
import java.util.function.Supplier;
import org.littletonrobotics.junction.*;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  public static final String CANIVORE_NAME = "CANivore";

  private final Autos autos;

  @SuppressWarnings("unused")
  private enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  @SuppressWarnings({"DataFlowIssue", "UnreachableCode", "ConstantValue"})
  public Robot() {
    // If this is erroring, hit build
    // Compiling auto-generates the BuildConstants file
    Logger.recordMetadata("Project", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("Git branch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("Git commit date", BuildConstants.GIT_DATE);
    Logger.recordMetadata("Git SHA", BuildConstants.GIT_SHA);
    //noinspection ConstantValue
    Logger.recordMetadata("Uncommited changes", BuildConstants.DIRTY == 1 ? "True" : "False");
    Logger.recordMetadata("Compile date", BuildConstants.BUILD_DATE);

    Mode mode = Robot.isReal() ? Mode.REAL : Mode.SIM;
    //    Mode mode = Robot.isReal() ? Mode.REAL : Mode.REPLAY;

    switch (mode) {
      case REAL -> {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        LoggedPowerDistribution.getInstance(); // Enables power distribution logging

        SignalLogger.start(); // Start logging Phoenix CAN signals
      }
      case SIM -> {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to the project's logs folder
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

        SignalLogger.start(); // Start logging Phoenix CAN signals
      }
      case REPLAY -> {
        setUseTiming(false); // Run as fast as possible
        String logPath;
        try {
          logPath =
              LogFileUtil
                  .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        } catch (Exception e) {
          throw new NoSuchElementException(
              "Failed to ask the user for a log file! If you are using IntelliJ, please open the "
                  + "log file in AdvantageScope and try again!");
        }
        // Note: User prompting will fail and crash on IntelliJ, so have the log open in AScope.
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(logPath, "_replay"))); // Save outputs to a new log
      }
    }

    Logger.start();

    var primary = new CommandXboxControllerWithRumble(0);

    var swerve = getSwerve(mode);
    var superstructure = getSuperstructure(mode);

    Supplier<Translation2d> driveTranslationalControlSupplier =
        () -> {
          double xControl = -primary.getLeftY();
          double yControl = -primary.getLeftX();
          double magnitude = Math.hypot(xControl, yControl);
          if (magnitude > 1) {
            xControl /= magnitude;
            yControl /= magnitude;
          } else if (magnitude > 1e-6) {
            double scalar = Math.pow(MathUtil.applyDeadband(magnitude, .06) / magnitude, 2);
            xControl *= scalar;
            yControl *= scalar;
          }
          return new Translation2d(xControl, yControl);
        };

    // Robot wants +X fwd, +Y left
    // Sticks are +X right +Y back
    swerve.setDefaultCommand(
        swerve.teleopDrive(
            driveTranslationalControlSupplier,
            () -> {
              var rawInput = MathUtil.applyDeadband(-primary.getRightX(), .06);
              return Math.copySign(rawInput * rawInput, rawInput);
            }));
    //    primary.povDown().whileTrue(swerve.stopInX());
    RobotModeTriggers.teleop().onTrue(swerve.resetGyro());
    RobotModeTriggers.disabled().whileTrue(swerve.stop());
    primary
        .leftTrigger()
        .and(superstructure::hasCoral)
        .whileTrue(swerve.reefAlign(true, driveTranslationalControlSupplier));
    primary
        .rightTrigger()
        .and(superstructure::hasCoral)
        .whileTrue(swerve.reefAlign(false, driveTranslationalControlSupplier));
    new Trigger(() -> swerve.nearSource() && !superstructure.hasCoral())
        .whileTrue(
            parallel(
                superstructure.intake(), swerve.sourceAlign(driveTranslationalControlSupplier)));
    //    primary.y().whileTrue(superstructure.lvl4(() -> true));
    //    primary.x().whileTrue(superstructure.lvl3(() -> true));
    //    primary.b().whileTrue(superstructure.lvl2(() -> true));
    //    primary.a().whileTrue(superstructure.lvl1());
    //    primary.rightBumper().whileTrue(superstructure.intake());

    autos = new Autos(swerve, superstructure);
    ReefLocations.log();
  }

  private Swerve getSwerve(Mode mode) {
    SwerveIO swerveIO;
    FiducialPoseEstimator.IO[] visionIOs;

    var cameraNames =
        new LoggableInputs() {
          String[] names;

          @Override
          public void toLog(LogTable table) {
            table.put("Names", names);
          }

          @Override
          public void fromLog(LogTable table) {
            names = table.get("Names", new String[0]);
          }
        };

    switch (mode) {
      case REAL, SIM -> {
        swerveIO = new SwerveIOPhoenix();
        visionIOs =
            new FiducialPoseEstimator.IO[] {
              new FiducialPoseEstimator.IO(
                  new FiducialPoseEstimatorIOPhoton(
                      "FrontLeft",
                      new Transform3d(
                          Units.inchesToMeters(22.75 / 2),
                          Units.inchesToMeters(22.75 / 2),
                          Units.inchesToMeters(8.25),
                          new Rotation3d(
                              0, Units.degreesToRadians(-15), Units.degreesToRadians(-30)))),
                  "FrontLeft"),
              //              new FiducialPoseEstimator.IO(
              //                  new FiducialPoseEstimatorIOPhoton(
              //                      "FrontRight",
              //                      new Transform3d(
              //                          Units.inchesToMeters(22.75 / 2),
              //                          -Units.inchesToMeters(22.75 / 2),
              //                          Units.inchesToMeters(8.25),
              //                          new Rotation3d(
              //                              0, Units.degreesToRadians(-15),
              // Units.degreesToRadians(0)))),
              //                  "FrontRight")
            };

        cameraNames.names = new String[visionIOs.length];
        for (int i = 0; i < visionIOs.length; i++) {
          cameraNames.names[i] = visionIOs[i].name();
        }
        Logger.processInputs("Vision", cameraNames);
      }
      default -> {
        swerveIO = new SwerveIO() {};
        Logger.processInputs("Vision", cameraNames);
        visionIOs = new FiducialPoseEstimator.IO[cameraNames.names.length];
        for (int i = 0; i < cameraNames.names.length; i++) {
          visionIOs[i] =
              new FiducialPoseEstimator.IO(new FiducialPoseEstimatorIO() {}, cameraNames.names[i]);
        }
      }
    }

    return new Swerve(swerveIO, visionIOs);
  }

  private Superstructure getSuperstructure(Mode mode) {
    switch (mode) {
      case REAL -> {
        return new Superstructure(new ElevatorIOPhoenix(), new CoralOuttakeIOPhoenix());
      }
      case SIM -> {
        return new Superstructure(new ElevatorIOPhoenix(), new CoralOuttakeIO() {});
      }
      default -> {
        return new Superstructure(new ElevatorIO() {}, new CoralOuttakeIO() {});
      }
    }
  }

  private Command autoCommand;

  @Override
  public void autonomousInit() {
    autoCommand = autos.getSelectedCommand();
    autoCommand.schedule();
  }

  @Override
  public void autonomousExit() {
    autoCommand.cancel();
    autos.clear();
  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);

    PhoenixBatchRefresher.refresh();
    // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled commands,
    // running already-scheduled commands, removing finished or interrupted commands, and running
    // subsystem periodic() methods. This must be called from the robot's periodic block in order
    // for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    Logger.recordOutput(
        "LoggedRobot/MemoryUsageMb",
        (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / 1e6);
    Logger.recordOutput("IsOnRed", isOnRed());
    SmartDashboard.putData(CommandScheduler.getInstance());

    Threads.setCurrentThreadPriority(false, 10);
  }

  @Override
  public void disabledPeriodic() {
    // Updates Choreo's auto selector.
    autos.update();
  }

  public static volatile double simVoltage = 12;

  public static Supplier<Pose2d> groundTruthPoseSupplier;

  @Override
  public void simulationPeriodic() {
    if (!Logger.hasReplaySource()) {
      if (simVoltage < RobotController.getBrownoutVoltage()) {
        throw new RuntimeException("Voltage too low! Terminating program to simulate brownout.");
      }
      RoboRioSim.setVInVoltage(simVoltage);
      if (groundTruthPoseSupplier != null) {
        Logger.recordOutput("Sim/Ground truth pose", groundTruthPoseSupplier.get());
      }
    }
  }

  public static boolean isOnRed() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red;
  }
}
