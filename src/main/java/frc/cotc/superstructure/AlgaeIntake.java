package frc.cotc.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

class AlgaeIntake extends SubsystemBase {
  private final AlgaeIntakeIO io;
  private final AlgaeIntakeIO.AlgaeIntakeIOInputs inputs = new AlgaeIntakeIO.AlgaeIntakeIOInputs();

  AlgaeIntake(AlgaeIntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Superstructure/AlgaeIntake", inputs);
  }

  Command intake() {
    return run(io::intake).finallyDo(io::brake);
  }

  Command outtake() {
    return run(io::outtake).finallyDo(io::brake);
  }

  boolean hasAlgae() {
    return inputs.hasAlgae;
  }
}
