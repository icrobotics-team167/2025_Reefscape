// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

/**
 * Calling {@link BaseStatusSignal#refreshAll(BaseStatusSignal...)} has a measurable increase in
 * performance due to reduced JNI and data read/write overhead compared to calling {@link
 * StatusSignal#refresh()} on individual signals.
 *
 * <p>To maximize performance, all {@link BaseStatusSignal}s that aren't on separate threads are
 * batched together into one call via a global state in this class.
 *
 * <p>Kinda ugly since it becomes way less clear when data is being updated and global state stuff
 * is hard to debug but sometimes you gotta prioritize the RoboRIO's happiness over your own.
 */
public final class PhoenixBatchRefresher {
  private PhoenixBatchRefresher() {}

  private static BaseStatusSignal[] registeredSignals = new BaseStatusSignal[0];

  public static void refresh() {
    BaseStatusSignal.refreshAll(registeredSignals);
  }

  public static void register(BaseStatusSignal... signals) {
    var newSignals = new BaseStatusSignal[registeredSignals.length + signals.length];
    System.arraycopy(registeredSignals, 0, newSignals, 0, registeredSignals.length);
    System.arraycopy(signals, 0, newSignals, registeredSignals.length, signals.length);
    registeredSignals = newSignals;
  }
}
