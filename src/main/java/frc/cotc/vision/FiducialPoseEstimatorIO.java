// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface FiducialPoseEstimatorIO {
  class FiducialPoseEstimatorIOInputs implements LoggableInputs {
    PoseEstimate[] poseEstimates = new PoseEstimate[0];

    @Override
    public void toLog(LogTable table) {
      for (int i = 0; i < poseEstimates.length; i++) {
        poseEstimates[i].toLog(table.getSubtable(String.valueOf(i)));
      }
    }

    @Override
    public void fromLog(LogTable table) {
      var list = new ArrayList<PoseEstimate>();
      int i = 0;
      while (true) {
        var estimate = PoseEstimate.fromLog(table.getSubtable(String.valueOf(i)));
        if (estimate.timestamp <= 0) {
          break;
        }
        list.add(estimate);
        i++;
      }
      poseEstimates = list.toArray(new PoseEstimate[i]);
    }

    public record PoseEstimate(Pose3d robotPoseEstimate, double timestamp, AprilTag[] tagsUsed) {
      public void toLog(LogTable table) {
        table.put("robotPoseEstimate", robotPoseEstimate);
        table.put("timestamp", timestamp);
        table.put("tagsUsed", AprilTag.struct, tagsUsed);
      }

      public static PoseEstimate fromLog(LogTable table) {
        return new PoseEstimate(
            table.get("robotPoseEstimate", Pose3d.kZero),
            table.get("timestamp", -1),
            table.get("tagsUsed", AprilTag.struct, new AprilTag[] {AprilTag.invalid}));
      }

      public record AprilTag(
          Pose3d location, int id, double distanceToCamera, double tx, double ty) {
        static final AprilTag invalid = new AprilTag(Pose3d.kZero, -1, -1, -1, -1);

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
                return Pose3d.struct.getSize() + kSizeInt32 + kSizeDouble * 3;
              }

              @Override
              public String getSchema() {
                return "Pose3d location;int32 id;double distanceToCamera;double tx;double ty";
              }

              @Override
              public AprilTag unpack(ByteBuffer bb) {
                var location = Pose3d.struct.unpack(bb);
                var id = bb.getInt();
                var distanceToCamera = bb.getDouble();
                var tx = bb.getDouble();
                var ty = bb.getDouble();
                return new AprilTag(location, id, distanceToCamera, tx, ty);
              }

              @Override
              public void pack(ByteBuffer bb, AprilTag value) {
                Pose3d.struct.pack(bb, value.location);
                bb.putInt(value.id);
                bb.putDouble(value.distanceToCamera);
                bb.putDouble(value.tx);
                bb.putDouble(value.ty);
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
}
