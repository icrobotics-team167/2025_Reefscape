package frc.cotc.arm;

import frc.cotc.util.MotorCurrentDraws;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface CoralOuttakeIO {
    @AutoLog
    class CoralOuttakeIOInputs implements LoggableInputs {

        public boolean coralIn = false;
        public MotorCurrentDraws motorCurrentDraws = new MotorCurrentDraws();

        @Override
        public void toLog(LogTable logTable) {
            logTable.put("coralIn", coralIn);
            logTable.put("motorCurrentDraws", MotorCurrentDraws.struct, motorCurrentDraws);
        }

        @Override
        public void fromLog(LogTable logTable) {

        }
    }

    void updateInputs(CoralOuttakeIOInputs inputs);

    void runVoltage(double volts);

    void brake();
}
