// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.vision;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.ArrayList;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface FiducialPoseEstimatorIO {
  class FiducialPoseEstimatorIOInputs implements LoggableInputs {
    public boolean hasNewData = false;
    public PoseEstimate[] poseEstimates = new PoseEstimate[0];

    @Override
    public void toLog(LogTable table) {
      table.put("hasNewData", hasNewData);
      for (int i = 0; i < poseEstimates.length; i++) {
        poseEstimates[i].toLog(table, i);
      }
    }

    @Override
    public void fromLog(LogTable table) {
      hasNewData = table.get("hasNewData", false);
      poseEstimates = PoseEstimate.fromLog(table);
    }
  }

  record PoseEstimate(Pose3d estimatedPose, double timestamp, Pose3d[] tagsUsed) {
    public void toLog(LogTable table, int i) {
      table.put("poseEstimates/" + i + "/estimatedPose", estimatedPose);
      table.put("poseEstimates/" + i + "/timestamp", timestamp);
      table.put("poseEstimates/" + i + "/tagsUsed", tagsUsed);
    }

    public static PoseEstimate[] fromLog(LogTable table) {
      var entries = new ArrayList<PoseEstimate>();

      int i = 0;
      while (true) {
        var timestamp = table.get("poseEstimates/" + i + "/timestamp", -1.0);
        if (timestamp < 0) {
          break;
        }

        var estimatedPose = table.get("poseEstimates/" + i + "/estimatedPose", Pose3d.kZero);
        var tagsUsed = table.get("poseEstimates/" + i + "/tagsUsed", new Pose3d[0]);

        entries.add(new PoseEstimate(estimatedPose, timestamp, tagsUsed));

        i++;
      }

      return entries.toArray(new PoseEstimate[i]);
    }
  }

  default void updateInputs(FiducialPoseEstimatorIOInputs inputs) {}
}
