// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
   public static final IUniqueRobotConstants uniqueRobotConstants = new TurtleShell();
  //public static final IUniqueRobotConstants uniqueRobotConstants = new TorinRobot();

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int outlevelbutton = 1;
    // public static final int buttonUnused1 = 1;
    public static final int toggleBreak2 = 2;
    public static final int bigArmBack = 3;
    public static final int littleArmBack = 4;
    public static final int bigArmForward = 5;
    public static final int littleArmFoward = 6;
    public static final int disableRotationButton = 7;
    public static final int stayPutCommandButton = 8;
    public static final int disableRotationButtonbottonunused = 9;
    public static final int enableRotationButton = 10;
    public static final int buttonUnused11 = 11;
    public static final int stayPutCommandButtonbottonunsed12 = 12;

    // left turtle ks=.16
  }

  public static class DriveTrainConstants {
    public static final int LEFT_1 = 18;
    public static final int LEFT_2 = 19;
    public static final int RIGHT_1 = 17;
    public static final int RIGHT_2 = 20;
  }

  public static class PDPConstants {
    public static final int ID = 1;
  }

  public static class GyroConstants {
    public static final int pigeonID = 50;
    public static final int NUM_TAPS = 10;
    public static final double balanceRange = 2.5;
  }

  public static class StayPutCommandConstants {
    public static final String p = "StayPut P";
    public static final String i = "StayPut I";
    public static final String d = "StayPut D";
    public static final String f = "StayPut F";
    public static final String enableTuning = "Enable StayPut Tuning";
  }

  public static class NumberConstants {
    public static final double WHEEL_DIAMETER = 6; // inches
    public static final double PI = 3.14;
  }

  public static class TurtleShell implements IUniqueRobotConstants {

    @Override
    public String getName() {
      // TODO Auto-generated method stub
      return "Turtle";
    }

    @Override
    public final MotorConstants getLeftDriveTrainMotorConstants() {
      return new MotorConstants(0, 0); // todo
    }

    @Override
    public final MotorConstants getRightDriveTrainMotorConstants() {
      return new MotorConstants(0, 0); // todo
    }

    @Override
    public final double getDistanceBetweenWheelsInches() {
      return 24.5;
    }

    @Override
    public boolean getDrivetraininversion() {

      return true;
    }

    @Override
    public double getDriveTrainVelocityConversion() {

      return 0.001388889;
    }

    @Override
    public double getDriveTrainPositionConversion() {
      return 1.7391;
    }
  }

  public static class TorinRobot implements IUniqueRobotConstants {

    @Override
    public String getName() {
      return "Torin";
    }

    @Override
    public final MotorConstants getLeftDriveTrainMotorConstants() {
      return new MotorConstants(0.1224, (1.8179824940435525*3.3));
    }

    @Override
    public final MotorConstants getRightDriveTrainMotorConstants() {
      return new MotorConstants(0.1364, (1.8601248134570176 *3.3));
    }

    @Override
    public final double getDistanceBetweenWheelsInches() {
      return 16;
    }

    @Override
    public boolean getDrivetraininversion() {

      return false;
    }

    @Override
    public double getDriveTrainVelocityConversion() {
      return .000402063138642;
    }

    @Override
    public double getDriveTrainPositionConversion() {
      return .0373987729;
    }
  }

  public static class PeristentMemory {

    public static final KeyValue[] list = {
      new KeyValue<Double>(StayPutCommandConstants.p, 0.0),
      new KeyValue<Double>(StayPutCommandConstants.i, 0.0),
      new KeyValue<Double>(StayPutCommandConstants.d, 0.0),
      new KeyValue<Double>(StayPutCommandConstants.f, 0.0),
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

  public interface IUniqueRobotConstants {
    public MotorConstants getLeftDriveTrainMotorConstants();

    public MotorConstants getRightDriveTrainMotorConstants();

    public String getName();

    public double getDistanceBetweenWheelsInches();
    public boolean getDrivetraininversion();

    public double getDriveTrainVelocityConversion();
    public double getDriveTrainPositionConversion();
  }

  public static class MotorConstants {
    public double ks;
    public double kv;
    public double ka;

    public MotorConstants(double ks, double kv) {
      this.ks = ks;
      this.kv = kv;
      this.ka = 0;
    }

    public MotorConstants(double ks, double kv, double ka) {
      this.ks = ks;
      this.kv = kv;
      this.ka = ka;
    }
  }
}
