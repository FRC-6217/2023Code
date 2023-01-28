// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.DoubleBuffer;
import java.util.ArrayList;
import java.util.Iterator;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kXboxDriver = 1;
    public static final int buttonUnused0 = 0;
    public static final int buttonUnused1 = 1;
    public static final int buttonUnused2 = 2;
    public static final int buttonUnused3 = 3;
    public static final int buttonUnused4 = 4;
    public static final int buttonUnused5 = 5;
    public static final int buttonUnused6 = 6;
    public static final int buttonUnused7 = 7;
    public static final int buttonUnused8 = 8;
    public static final int disableRotationButton = 9;
    public static final int enableRotationButton = 10;
    public static final int buttonUnused11 = 11;
    public static final int stayPutCommandButton = 12;


  }

  public static class DriveTrainConstants {
    public static final int LEFT_1 = 40;
    public static final int LEFT_2 = 41;
    public static final int RIGHT_1 = 42;
    public static final int RIGHT_2 = 43;
  }

  public static class PDPConstants {
    public static final int ID = 1;
  }

  public static class GyroConstants {
    public static final int pigeonID = 10;
    public static final int NUM_TAPS = 10;
  }

  public static class StayPutCommandConstants {
    public static final String p = "StayPut P";
    public static final String i = "StayPut I";
    public static final String d = "StayPut D";
    public static final String enableTuning = "Enable StayPut Tuning";
  }

  public static class NumberConstants {
    public static final double WHEEL_DIAMETER = 6;//inches
    public static final double PI = 3.14;
  }

  public static class PeristentMemory {

    public static final KeyValue[] list = {
      new KeyValue<Double>(StayPutCommandConstants.p, 0.0),
      new KeyValue<Double>(StayPutCommandConstants.i, 0.0),
      new KeyValue<Double>(StayPutCommandConstants.d, 0.0),
      new KeyValue<Boolean>(StayPutCommandConstants.enableTuning, true),

    };

  }

  public static class KeyValue<T> {
    public String key;
    public T value;

    public KeyValue(String key, T value) {
        this.key = key;
        this.value = value;
    }

  }
}
