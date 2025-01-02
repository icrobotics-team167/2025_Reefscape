// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.cotc.drive.Swerve;
import frc.cotc.drive.SwerveIO;
import frc.cotc.drive.SwerveIOPhoenix;
import frc.cotc.vision.FiducialPoseEstimatorIO;
import frc.cotc.vision.FiducialPoseEstimatorIOPhoton;
import java.util.Arrays;
import java.util.NoSuchElementException;
import java.util.function.Supplier;
import org.littletonrobotics.junction.*;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  public static final String CANIVORE_NAME = "Canivore";

  private final Autos autos;

  @SuppressWarnings("unused")
  private enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  @SuppressWarnings({"DataFlowIssue", "UnreachableCode"})
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

    Swerve swerve = getSwerve(mode);

    var primary = new CommandXboxController(0);

    // Robot wants +X fwd, +Y left
    // Sticks are +X right +Y back
    swerve.setDefaultCommand(
        swerve.teleopDrive(
            () -> -primary.getLeftY(),
            () -> -primary.getLeftX(),
            .06,
            2,
            () -> -primary.getRightX(),
            .05,
            2));
    RobotModeTriggers.disabled().or(primary.povDown()).whileTrue(swerve.stopInX());
    RobotModeTriggers.teleop().onTrue(swerve.resetGyro());

    autos = new Autos(swerve);
  }

  private Swerve getSwerve(Mode mode) {
    SwerveIO swerveIO;
    FiducialPoseEstimatorIO[] visionIOs;

    var count =
        new LoggableInputs() {
          int cameraCount = 0;

          @Override
          public void toLog(LogTable table) {
            table.put("cameraCount", cameraCount);
          }

          @Override
          public void fromLog(LogTable table) {
            cameraCount = table.get("cameraCount", 0);
          }
        };

    switch (mode) {
      case REAL, SIM -> {
        swerveIO = new SwerveIOPhoenix();
        visionIOs =
            new FiducialPoseEstimatorIO[] {
              new FiducialPoseEstimatorIOPhoton(0),
              new FiducialPoseEstimatorIOPhoton(1),
              new FiducialPoseEstimatorIOPhoton(2),
              new FiducialPoseEstimatorIOPhoton(3)
            };
        count.cameraCount = visionIOs.length;
        Logger.processInputs("Vision", count);
      }
      default -> {
        Logger.processInputs("Vision", count);

        swerveIO = new SwerveIO() {};
        visionIOs = new FiducialPoseEstimatorIO[count.cameraCount];
        Arrays.fill(visionIOs, new FiducialPoseEstimatorIO() {});
      }
    }

    return new Swerve(swerveIO, visionIOs);
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
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled commands,
    // running already-scheduled commands, removing finished or interrupted commands, and running
    // subsystem periodic() methods. This must be called from the robot's periodic block in order
    // for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    Logger.recordOutput(
        "LoggedRobot/MemoryUsageMb",
        (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / 1e6);
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
      Logger.recordOutput("Sim/Ground truth pose", groundTruthPoseSupplier.get());
    }
  }

  public static boolean isOnRed() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red;
  }
}
