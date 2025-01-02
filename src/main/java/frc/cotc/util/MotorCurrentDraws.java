// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.util;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class MotorCurrentDraws {
  public double statorCurrent;
  public double supplyCurrent;

  public MotorCurrentDraws(double stator, double supply) {
    statorCurrent = stator;
    supplyCurrent = supply;
  }

  public static MotorCurrentDraws fromSignals(
      BaseStatusSignal statorSignal, BaseStatusSignal supplySignal) {
    return new MotorCurrentDraws(statorSignal.getValueAsDouble(), supplySignal.getValueAsDouble());
  }

  /** MUTABLE SHIT IS VERY FOOT-GUN-Y. BE CAREFUL. */
  public void mutateFromSignals(BaseStatusSignal statorSignal, BaseStatusSignal supplySignal) {
    statorCurrent = statorSignal.getValueAsDouble();
    supplyCurrent = supplySignal.getValueAsDouble();
  }

  public static Struct<MotorCurrentDraws> struct =
      new Struct<>() {
        @Override
        public Class<MotorCurrentDraws> getTypeClass() {
          return MotorCurrentDraws.class;
        }

        @Override
        public String getTypeName() {
          return "MotorCurrentDraws";
        }

        @Override
        public int getSize() {
          return kSizeDouble * 2;
        }

        @Override
        public String getSchema() {
          return "double stator;double supply";
        }

        @Override
        public MotorCurrentDraws unpack(ByteBuffer bb) {
          var stator = bb.getDouble();
          var supply = bb.getDouble();
          return new MotorCurrentDraws(stator, supply);
        }

        @Override
        public void pack(ByteBuffer bb, MotorCurrentDraws value) {
          bb.putDouble(value.statorCurrent);
          bb.putDouble(value.supplyCurrent);
        }
      };
}
