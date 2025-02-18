// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.cotc.arm.ElevatorIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
    private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter/feeder", inputs);
    }

    public Command goL1() {
        return startEnd(io::setVoltage, io::stop);
    }

    public Command goL2() {
        return startEnd(io::setVoltage, io::stop);
    }

    public Command goL3() {
        return startEnd(io::setVoltage, io::stop);
    }

    public Command goL4() {
        return startEnd(io::setVoltage, io::stop);
    }
}
