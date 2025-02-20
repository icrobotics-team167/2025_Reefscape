package frc.cotc.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralOuttake extends SubsystemBase {

    private final CoralOuttakeIO io;
    private final CoralOuttakeIO.CoralOuttakeIOInputs inputs = new CoralOuttakeIO.CoralOuttakeIOInputs();

    public CoralOuttake(CoralOuttakeIO io){
        this.io = io;
    }

    public Command in(){
        return runOnce(() -> io.runVoltage(5));
    }

    public Command out(){
        return runOnce(() -> io.runVoltage(8));
    }

    public Command stop(){
        return runOnce(() -> io.runVoltage(0));
    }
}
