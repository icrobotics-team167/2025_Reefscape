// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.cotc.algae.*;
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
import java.util.*;
import java.util.function.DoubleSupplier;
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

  public static boolean isNewBot;

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

    var newBotLogger =
        new LoggableInputs() {
          boolean isNewBot;

          @Override
          public void toLog(LogTable table) {
            table.put("isNewBot", isNewBot);
          }

          @Override
          public void fromLog(LogTable table) {
            isNewBot = table.get("isNewBot", true);
          }
        };
    switch (mode) {
      case REAL -> {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        LoggedPowerDistribution.getInstance(); // Enables power distribution logging

        var serialNumber = RobotController.getSerialNumber();
        Logger.recordMetadata("RoboRIO Serial number", serialNumber);

        SignalLogger.start(); // Start logging Phoenix CAN signals
        newBotLogger.isNewBot = serialNumber.equals("032BE4AB");
      }
      case SIM -> {
        Logger.addDataReceiver(new WPILOGWriter()); // Log to the project's logs folder
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

        SignalLogger.start(); // Start logging Phoenix CAN signals
        newBotLogger.isNewBot = true;
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

    Logger.processInputs("Robot", newBotLogger);
    isNewBot = newBotLogger.isNewBot;

    var primary = new CommandXboxControllerWithRumble(0);
    var secondary = new CommandXboxControllerWithRumble(1);

    var superstructure = getSuperstructure(mode);
    var swerve = getSwerve(mode, superstructure::getElevatorExtension);
    var algaePivot =
        new AlgaePivot(
            Robot.isReal() && isNewBot ? new AlgaePivotIOPhoenix() : new AlgaePivotIO() {});
    var algaeIntake =
        new AlgaeIntake(
            Robot.isReal() && isNewBot ? new AlgaeIntakeIOPhoenix() : new AlgaeIntakeIO() {});

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

    RobotModeTriggers.teleop().onTrue(swerve.resetGyro());
    RobotModeTriggers.disabled().whileTrue(swerve.stop());

    // Robot wants +X fwd, +Y left
    // Sticks are +X right +Y back
    swerve.setDefaultCommand(
        swerve.teleopDrive(
            driveTranslationalControlSupplier,
            () -> {
              var rawInput = MathUtil.applyDeadband(-primary.getRightX(), .06);
              return Math.copySign(rawInput * rawInput, rawInput);
            },
            primary.leftBumper()));
    primary
        .leftTrigger()
        .whileTrue(swerve.reefBranchAlign(true, driveTranslationalControlSupplier));
    primary
        .rightTrigger()
        .whileTrue(swerve.reefBranchAlign(false, driveTranslationalControlSupplier));
    primary.a().whileTrue(swerve.reefAlgaeAlign(driveTranslationalControlSupplier));
    primary.b().whileTrue(swerve.netAlign(driveTranslationalControlSupplier));
    primary.povLeft().debounce(.5).onTrue(superstructure.readyClimb());
    primary.povUp().and(superstructure::isClimberDeployed).whileTrue(superstructure.climb());
    primary
        .povDown()
        .and(superstructure::isClimberDeployed)
        .whileTrue(superstructure.raiseClimber());

    //        primary.b().whileTrue(swerve.testSlipCurrent());
    //    primary.b().onTrue(swerve.lockForward()).onFalse(swerve.lockBackwards());

    secondary
        .y()
        .whileTrue(
            waitUntil(() -> swerve.nearingTargetPose() || secondary.getHID().getRightBumperButton())
                .andThen(
                    superstructure.lvl4(
                        () ->
                            swerve.atTargetPoseTeleop()
                                || secondary.getHID().getRightBumperButton())));
    secondary
        .x()
        .whileTrue(
            waitUntil(() -> swerve.nearingTargetPose() || secondary.getHID().getRightBumperButton())
                .andThen(
                    superstructure.lvl3(
                        () ->
                            swerve.atTargetPoseTeleop()
                                || secondary.getHID().getRightBumperButton())));
    secondary
        .b()
        .whileTrue(
            waitUntil(() -> swerve.nearingTargetPose() || secondary.getHID().getRightBumperButton())
                .andThen(
                    superstructure.lvl2(
                        () ->
                            swerve.atTargetPoseTeleop()
                                || secondary.getHID().getRightBumperButton())));

    algaePivot.setDefaultCommand(algaePivot.manualControl(() -> -secondary.getLeftY()));
    secondary.leftTrigger().or(algaeIntake::hasAlgae).whileTrue(algaeIntake.intake());
    secondary
        .leftTrigger()
        .whileTrue(
            either(
                superstructure.highAlgae(),
                none(),
                () -> {
                  var angle = swerve.getReefAlignPose().getRotation();
                  if (Robot.isOnRed()) {
                    return angle.equals(Rotation2d.kPi)
                        || angle.equals(Rotation2d.fromDegrees(60))
                        || angle.equals(Rotation2d.fromDegrees(-60));
                  } else {
                    return angle.equals(Rotation2d.kZero)
                        || angle.equals(Rotation2d.fromDegrees(120))
                        || angle.equals(Rotation2d.fromDegrees(-120));
                  }
                }));
    secondary.leftBumper().whileTrue(algaeIntake.outtake());
    secondary.rightTrigger().whileTrue(superstructure.net());

    superstructure.coralStuck().debounce(.25).onTrue(superstructure.ejectStuckCoral());

    new Trigger(superstructure::hasCoral).onChange(secondary.rumble(.35));
    new Trigger(algaeIntake::hasAlgae).onChange(secondary.rumble(.35));

    autos = new Autos(swerve, superstructure);
    ReefLocations.log();

    CommandScheduler.getInstance().onCommandInitialize(this::commandStarted);
    CommandScheduler.getInstance().onCommandFinish(this::commandEnded);
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (interrupted, interrupting) -> {
              interrupting.ifPresent(
                  interrupter -> runningInterrupters.put(interrupter, interrupted));
              commandEnded(interrupted);
            });
  }

  private final Set<Command> runningNonInterrupters = new HashSet<>();
  private final Map<Command, Command> runningInterrupters = new HashMap<>();
  private final Map<Subsystem, Command> requiredSubsystems = new HashMap<>();

  private void commandStarted(final Command command) {
    if (!runningInterrupters.containsKey(command)) {
      runningNonInterrupters.add(command);
    }

    for (final Subsystem subsystem : command.getRequirements()) {
      requiredSubsystems.put(subsystem, command);
    }
  }

  private void commandEnded(final Command command) {
    runningNonInterrupters.remove(command);
    runningInterrupters.remove(command);

    for (final Subsystem subsystem : command.getRequirements()) {
      requiredSubsystems.remove(subsystem);
    }
  }

  private final StringBuilder subsystemsBuilder = new StringBuilder();

  private String getCommandName(Command command) {
    subsystemsBuilder.setLength(0);
    int j = 1;
    for (final Subsystem subsystem : command.getRequirements()) {
      subsystemsBuilder.append(subsystem.getName());
      if (j < command.getRequirements().size()) {
        subsystemsBuilder.append(",");
      }

      j++;
    }
    var finalName = command.getName();
    if (j > 1) {
      finalName += " (" + subsystemsBuilder + ")";
    }
    return finalName;
  }

  private void logRunningCommands() {
    Logger.recordOutput("CommandScheduler/Running/.type", "Alerts");

    //    final String[] runningCommands = new String[runningNonInterrupters.size()];
    //    int i = 0;
    //    for (final Command command : runningNonInterrupters) {
    //      runningCommands[i] = getCommandName(command);
    //      i++;
    //    }
    final ArrayList<String> runningCommands = new ArrayList<>();
    final ArrayList<String> runningDefaultCommands = new ArrayList<>();
    for (final Command command : runningNonInterrupters) {
      boolean isDefaultCommand = false;
      for (Subsystem subsystem : command.getRequirements()) {
        if (subsystem.getDefaultCommand() == command) {
          runningDefaultCommands.add(getCommandName(command));
          isDefaultCommand = true;
          break;
        }
      }
      if (!isDefaultCommand) {
        runningCommands.add(getCommandName(command));
      }
    }
    Logger.recordOutput(
        "CommandScheduler/Running/warnings", runningCommands.toArray(new String[0]));
    Logger.recordOutput(
        "CommandScheduler/Running/infos", runningDefaultCommands.toArray(new String[0]));

    final String[] interrupters = new String[runningInterrupters.size()];
    int j = 0;
    for (final Map.Entry<Command, Command> entry : runningInterrupters.entrySet()) {
      final Command interrupter = entry.getKey();
      final Command interrupted = entry.getValue();

      interrupters[j] = getCommandName(interrupter) + " interrupted " + getCommandName(interrupted);
      j++;
    }

    Logger.recordOutput("CommandScheduler/Running/errors", interrupters);
  }

  private void logRequiredSubsystems() {
    Logger.recordOutput("CommandScheduler/Subsystems/.type", "Alerts");

    final String[] subsystems = new String[requiredSubsystems.size()];
    {
      int i = 0;
      for (final Map.Entry<Subsystem, Command> entry : requiredSubsystems.entrySet()) {
        final Subsystem required = entry.getKey();
        final Command command = entry.getValue();

        subsystems[i] = required.getName() + " (" + command.getName() + ")";
        i++;
      }
    }
    Logger.recordOutput("CommandScheduler/Subsystems/infos", subsystems);
  }

  private Swerve getSwerve(Mode mode, DoubleSupplier elevatorExtensionSupplier) {
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

        if (isNewBot || Robot.isSimulation()) {
          visionIOs =
              new FiducialPoseEstimator.IO[] {
                new FiducialPoseEstimator.IO(
                    new FiducialPoseEstimatorIOPhoton(
                        "NewFrontLeft",
                        new Transform3d(
                            Units.inchesToMeters(22.75 / 2 - 1.5),
                            Units.inchesToMeters(22.75 / 2 + .125),
                            Units.inchesToMeters(8.375),
                            new Rotation3d(
                                0, Units.degreesToRadians(-15), Units.degreesToRadians(-30)))),
                    "NewFrontLeft"),
                new FiducialPoseEstimator.IO(
                    new FiducialPoseEstimatorIOPhoton(
                        "NewFrontRight",
                        new Transform3d(
                            Units.inchesToMeters(22.75 / 2 - 1.5),
                            -Units.inchesToMeters(22.75 / 2 + .125),
                            Units.inchesToMeters(8.375),
                            new Rotation3d(
                                0, Units.degreesToRadians(-15), Units.degreesToRadians(30)))),
                    "NewFrontRight")
              };
        } else {
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
                new FiducialPoseEstimator.IO(
                    new FiducialPoseEstimatorIOPhoton(
                        "FrontRight",
                        new Transform3d(
                            Units.inchesToMeters(22.75 / 2),
                            -Units.inchesToMeters(22.75 / 2),
                            Units.inchesToMeters(8.25),
                            new Rotation3d(
                                0, Units.degreesToRadians(-15), Units.degreesToRadians(30)))),
                    "FrontRight")
              };
        }

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

    return new Swerve(swerveIO, visionIOs, elevatorExtensionSupplier);
  }

  private Superstructure getSuperstructure(Mode mode) {
    switch (mode) {
      case REAL -> {
        return isNewBot
            ? new Superstructure(
                new ElevatorIOPhoenix(),
                new CoralOuttakeIOPhoenix(),
                new RampIOPhoenix(),
                new ClimberIOPhoenix())
            : new Superstructure(
                new ElevatorIO() {}, new CoralOuttakeIO() {}, new RampIO() {}, new ClimberIO() {});
      }
      case SIM -> {
        return new Superstructure(
            new ElevatorIOPhoenix(),
            new CoralOuttakeIOSim(),
            new RampIOPhoenix(),
            new ClimberIO() {});
      }
      default -> {
        return new Superstructure(
            new ElevatorIO() {}, new CoralOuttakeIO() {}, new RampIO() {}, new ClimberIO() {});
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
    logRunningCommands();
    logRequiredSubsystems();
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
