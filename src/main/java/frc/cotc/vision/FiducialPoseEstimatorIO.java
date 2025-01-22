package frc.cotc.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface FiducialPoseEstimatorIO {
  class FiducialPoseEstimatorIOInputs implements LoggableInputs {
    @Override
    public void toLog(LogTable table) {

    }

    @Override
    public void fromLog(LogTable table) {

    }

    public record PoseEstimate() {}
  }

  default void updateInputs(FiducialPoseEstimatorIOInputs inputs) {}
}
