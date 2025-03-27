// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.cotc.util.MotorCurrentDraws;
import java.nio.ByteBuffer;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface SwerveIO {
  class SwerveIOInputs implements LoggableInputs {
    SwerveModuleState[] moduleStates =
        new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
        };
    Rotation2d gyroYaw = Rotation2d.kZero;

    OdometryFrame[] odometryFrames =
        new OdometryFrame[] {
          new OdometryFrame(
              new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
              },
              Rotation2d.kZero,
              -1.0)
        };

    MotorCurrentDraws[] driveMotorCurrents =
        new MotorCurrentDraws[] {
          new MotorCurrentDraws(),
          new MotorCurrentDraws(),
          new MotorCurrentDraws(),
          new MotorCurrentDraws()
        };
    MotorCurrentDraws[] steerMotorCurrents =
        new MotorCurrentDraws[] {
          new MotorCurrentDraws(),
          new MotorCurrentDraws(),
          new MotorCurrentDraws(),
          new MotorCurrentDraws()
        };

    @Override
    public void toLog(LogTable table) {
      table.put("ModuleStates", moduleStates);
      table.put("GyroYaw", gyroYaw);
      table.put("OdometryFrames", OdometryFrame.struct, odometryFrames);
      table.put("DriveMotorCurrents", MotorCurrentDraws.struct, driveMotorCurrents);
      table.put("SteerMotorCurrents", MotorCurrentDraws.struct, steerMotorCurrents);
    }

    @Override
    public void fromLog(LogTable table) {
      moduleStates = table.get("ModuleStates", new SwerveModuleState[4]);
      gyroYaw = table.get("GyroYaw", Rotation2d.kZero);
      odometryFrames = table.get("OdometryFrames", OdometryFrame.struct);
      driveMotorCurrents =
          table.get("DriveMotorCurrents", MotorCurrentDraws.struct, new MotorCurrentDraws[4]);
      steerMotorCurrents =
          table.get("SteerMotorCurrents", MotorCurrentDraws.struct, new MotorCurrentDraws[4]);
    }
  }

  record OdometryFrame(SwerveModulePosition[] positions, Rotation2d gyroYaw, double timestamp)
      implements StructSerializable {
    public OdometryFrame {
      if (positions == null || positions.length != 4) {
        throw new IllegalArgumentException("Position count not 4");
      }
      if (gyroYaw == null) {
        throw new NullPointerException("Gyro yaw is null");
      }
    }

    public static final Struct<OdometryFrame> struct =
        new Struct<>() {
          @Override
          public Class<OdometryFrame> getTypeClass() {
            return OdometryFrame.class;
          }

          @Override
          public String getTypeName() {
            return "OdometryFrame";
          }

          @Override
          public int getSize() {
            return SwerveModulePosition.struct.getSize() * 4
                + Rotation2d.struct.getSize()
                + kSizeDouble;
          }

          @Override
          public String getSchema() {
            return "SwerveModulePosition positions[4];Rotation2d gyroYaw;double timestamp";
          }

          @Override
          public Struct<?>[] getNested() {
            return new Struct<?>[] {SwerveModulePosition.struct, Rotation2d.struct};
          }

          @Override
          public OdometryFrame unpack(ByteBuffer bb) {
            var positions = Struct.unpackArray(bb, 4, SwerveModulePosition.struct);
            var yaw = Rotation2d.struct.unpack(bb);
            var timestamp = bb.getDouble();
            return new OdometryFrame(positions, yaw, timestamp);
          }

          @Override
          public void pack(ByteBuffer bb, OdometryFrame value) {
            Struct.packArray(bb, value.positions, SwerveModulePosition.struct);
            Rotation2d.struct.pack(bb, value.gyroYaw);
            bb.putDouble(value.timestamp);
          }
        };
  }

  @SuppressWarnings("CanBeFinal")
  @AutoLog
  class SwerveModuleConstants {
    // Meters
    double TRACK_WIDTH_METERS = 1;
    double TRACK_LENGTH_METERS = 1;
    double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);

    // Should have a gear reduction applied with .withReduction()
    DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60(1).withReduction(6.75);
    int SLIP_CURRENT_AMPS = 80;

    double MASS_KG = 40;
    double MOI_KG_METERS_SQUARED = 40;

    // Should already have a reduction applied
    double[] MAX_STEER_SPEEDS_RAD_PER_SEC =
        new double[] {
          Units.rotationsPerMinuteToRadiansPerSecond(6000) / (150.0 / 7.0),
          Units.rotationsPerMinuteToRadiansPerSecond(6000) / (150.0 / 7.0),
          Units.rotationsPerMinuteToRadiansPerSecond(6000) / (150.0 / 7.0),
          Units.rotationsPerMinuteToRadiansPerSecond(6000) / (150.0 / 7.0)
        };

    // Due to kinematic limits, it may not be possible for the bot to stay moving straight when
    // spinning and moving at the same time. This fudge factor slows down the max angular speed
    // when the bot is translating, but doesn't affect the limit when the bot isn't translating.
    // Scalar
    double ANGULAR_SPEED_FUDGING = .45;
  }

  /**
   * Gets the drive constants.
   *
   * @return A SwerveIOConstantsAutoLogged object that contains constants.
   */
  default SwerveModuleConstantsAutoLogged getConstants() {
    return new SwerveModuleConstantsAutoLogged();
  }

  /**
   * Updates the {@link SwerveIOInputs} to feed in new data from the drivebase.
   *
   * @param inputs The SwerveIOInputs object. Will be mutated.
   */
  default void updateInputs(SwerveIOInputs inputs) {}

  default void drive(SwerveSetpointGenerator.SwerveSetpoint setpoint) {}

  default void stop(Rotation2d[] angles) {}

  default void resetGyro(Rotation2d newYaw) {}

  default void testSlipCurrent(double amps) {}
}
