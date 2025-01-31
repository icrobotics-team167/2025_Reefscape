// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface FiducialPoseEstimatorIO {
  @AutoLog
  class FiducialPoseEstimatorIOConstants {
    Transform3d robotToCameraTransform = Transform3d.kZero;
  }

  class FiducialPoseEstimatorIOInputs implements LoggableInputs {
    FiducialPoseEstimate[] poseEstimates = new FiducialPoseEstimate[0];

    @Override
    public void toLog(LogTable table) {
      for (int i = 0; i < poseEstimates.length; i++) {
        poseEstimates[i].toLog(table, String.valueOf(i));
      }
    }

    @Override
    public void fromLog(LogTable table) {
      var list = new ArrayList<FiducialPoseEstimate>();
      int i = 0;
      while (true) {
        var estimate = FiducialPoseEstimate.fromLog(table, String.valueOf(i));
        if (estimate.timestamp <= 0) {
          break;
        }
        list.add(estimate);
        i++;
      }
      poseEstimates = list.toArray(new FiducialPoseEstimate[i]);
    }

    public record FiducialPoseEstimate(
        Pose3d robotPoseEstimate, double timestamp, AprilTag[] tagsUsed) {
      public void toLog(LogTable table, String subtable) {
        table.put(subtable + "/robotPoseEstimate", robotPoseEstimate);
        table.put(subtable + "/timestamp", timestamp);
        table.put(subtable + "/tagsUsed", AprilTag.struct, tagsUsed);
      }

      public static FiducialPoseEstimate fromLog(LogTable table, String subtable) {
        return new FiducialPoseEstimate(
            table.get(subtable + "/robotPoseEstimate", Pose3d.kZero),
            table.get(subtable + "/timestamp", -1.0),
            table.get(subtable + "/tagsUsed", AprilTag.struct, new AprilTag[] {AprilTag.invalid}));
      }

      public record AprilTag(
          Pose3d location,
          int id,
          double distanceToCamera,
          double tx,
          double ty,
          double ambiguity) {
        static final AprilTag invalid = new AprilTag(Pose3d.kZero, -1, -1, -1, -1, -1);

        public static final Struct<AprilTag> struct =
            new Struct<>() {
              @Override
              public Class<AprilTag> getTypeClass() {
                return AprilTag.class;
              }

              @Override
              public String getTypeName() {
                return "AprilTag";
              }

              @Override
              public int getSize() {
                return Pose3d.struct.getSize() + kSizeInt32 + kSizeDouble * 4;
              }

              @Override
              public String getSchema() {
                return "Pose3d location;int32 id;double distanceToCamera;double tx;"
                    + "double ty;double ambiguity";
              }

              @Override
              public AprilTag unpack(ByteBuffer bb) {
                var location = Pose3d.struct.unpack(bb);
                var id = bb.getInt();
                var distanceToCamera = bb.getDouble();
                var tx = bb.getDouble();
                var ty = bb.getDouble();
                var ambiguity = bb.getDouble();
                return new AprilTag(location, id, distanceToCamera, tx, ty, ambiguity);
              }

              @Override
              public void pack(ByteBuffer bb, AprilTag value) {
                Pose3d.struct.pack(bb, value.location);
                bb.putInt(value.id);
                bb.putDouble(value.distanceToCamera);
                bb.putDouble(value.tx);
                bb.putDouble(value.ty);
                bb.putDouble(value.ambiguity);
              }

              @Override
              public Struct<?>[] getNested() {
                return new Struct<?>[] {Pose3d.struct};
              }
            };
      }
    }
  }

  default void updateInputs(FiducialPoseEstimatorIOInputs inputs) {}

  default FiducialPoseEstimatorIOConstantsAutoLogged getConstants() {
    return new FiducialPoseEstimatorIOConstantsAutoLogged();
  }
}
