package frc.cotc.superstructure;

import frc.cotc.util.MotorCurrentDraws;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface AlgaeIntakeIO {
  class AlgaeIntakeIOInputs implements LoggableInputs {
    boolean hasAlgae;

    MotorCurrentDraws currentDraws = new MotorCurrentDraws();

    @Override
    public void toLog(LogTable table) {
      table.put("hasAlgae", hasAlgae);
      table.put("currentDraws", MotorCurrentDraws.struct, currentDraws);
    }

    @Override
    public void fromLog(LogTable table) {
      hasAlgae = table.get("hasAlgae", false);
      currentDraws = table.get("currentDraws", MotorCurrentDraws.struct, currentDraws);
    }
  }

  default void updateInputs(AlgaeIntakeIOInputs inputs) {}

  default void intake() {}

  default void outtake() {}

  default void brake() {}
}
