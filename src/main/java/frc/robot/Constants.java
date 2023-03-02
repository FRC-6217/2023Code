// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.ArmSystem.ArmPosition;
import frc.robot.subsystems.ArmSystem.Position;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //public static final IUniqueRobotConstants uniqueRobotConstants = new TurtleShell();
  public static final IUniqueRobotConstants uniqueRobotConstants = new TorinRobot();

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int buttonBoxPort = 1;

    public static final int stayPut1 = 1;
    public static final int toggleBreak2 = 2;
    public static final int buttonUnused3 = 3;
    public static final int buttonUnused4 = 4;
    public static final int buttonUnused5 = 5;
    public static final int fullBalanceAct6 = 6;
    public static final int buttonUnused7 = 7;
    public static final int buttonUnused8 = 8;
    public static final int buttonUnused9 = 9;
    public static final int doAutoBalance10 = 10;
    public static final int cancelDrive11 = 11;
    public static final int toggleTurning12 = 12;


    public static final double deadBandX = 0.1;
    public static final double deadBandY = 0.1;

    public static final double defaultRotationGoverner = 0.5;
    

    // left turtle ks=.16
  }

  public static class PneumaticConstants {
      public static final String compressorKey = "compressor ";
      public static class Claw {
        public static final int clawChannelForwards = 0;
        public static final int clawChannelBackwards = 2;
        public static final String key = "Claw: ";
      }
      public static class BigArmBrake {
        public static final int channelForwards = 3;
        public static final int channelBackwards = 4;
        public static final String key = "BigArmBrake: ";
      }
  }

  public static class ArmSystemConstants {
    public static final int bigArmCANID = 11;
    public static final int littleArmCANID = 10;
    public static final int bigArmZeroChannel = 9;
    public static final int littleArmPotAnalogInChannel = 3;
    public static final int DMA_DIO_INPUT_CHANNEL = 0;

    public static class LittleArmAngle{
      
    public static final double Home = 0;
    public static final double ObjectSafety = 1;
    public static final double GroundPickUp = 0;
    public static final double SubstationPickUp = 0;
    public static final double HighSafteyPositionFront = 0;
    public static final double HighSafteyPositionBack = 0;
    public static final double High_Drop_Off_Cone = 0;
    public static final double High_Drop_Off_Cube = 0;
    public static final double Mid_Drop_Off_Cone = 0;
    public static final double Mid_Drop_Off_Cube = 0;
    public static final double Low_Drop_Off_Both = 0;
    public static final double Safe_Position = 0;
    public static final double Home_Position = 0;

    public static final double Pvalue = 0;
    public static final double Ivalue = 0;
    public static final double Dvalue = 0;

    public static final double maxAngle = 0;
    public static final double minAngle = 0;

    }

    public static class BigArmAngle{

        public static final double Home = 0;
        public static final double ObjectSafety = 0;
        public static final double GroundPickUp = 0;
        public static final double SubstationPickUp = 0;
        public static final double HighSafteyPositionFront = 0;
        public static final double HighSafteyPositionBack = 0;
        public static final double High_Drop_Off_Cone = 0;
        public static final double High_Drop_Off_Cube = 0;
        public static final double Mid_Drop_Off_Cone = 0;
        public static final double Mid_Drop_Off_Cube = 0;
        public static final double Low_Drop_Off_Both = 0;
        public static final double Safe_Position = 0;
        public static final double Home_Position = 0;

        public static final double Ivalue = 0;
        public static final double Dvalue = 0;
        public static final double Pvalue = 0;

        public static final double maxAngle = 40;
        public static final double minAngle = -50;
    }
  }

  public static class DriveTrainConstants {
    public static final int LEFT_1 = 18;
    public static final int LEFT_2 = 19;
    public static final int RIGHT_1 = 17;
    public static final int RIGHT_2 = 20;
    public static final double rampSpeedInSeconds = 0.75;
  }

  public static class PDPConstants {
    public static final int ID = 1;
  }

  public static class GyroConstants {
    public static final int pigeonID = 50;
    public static final double balanceRange = 2.5;
  }

  public static class StayPutCommandConstants {
    public static final String p = "StayPut P";
    public static final String i = "StayPut I";
    public static final String d = "StayPut D";
    public static final String f = "StayPut F";
    public static final String enableTuning = "Enable StayPut Tuning";

    public static double P = 2.5;
    public static double I = 0;
    public static double D = 0.1;
  }

  public static class BalanceConstants {
    public static final double debounceTime = 0.5;
  }

  public static class NumberConstants {
    public static final double WHEEL_DIAMETER = 6; // inches
    public static final double PI = 3.14;
  }

  public static class AutoConstants {
    public static final double leftLeaveSpeed = 0.7;
    public static final double rightLeaveSpeed = 0.7;
    public static final double leftLeaveDistanceInches = -168;
    public static final double rightLeaveDistanceInches = -168;
    public static final double middleLeaveOnPlatformInches = -84;
    public static final double middleLeaveOffPlatformInches = -50;
    public static final double middleLeaveOnPlatformSpeed = .7;
    public static final double middleLeaveOffPlatformSpeed = .7;
    public static final double AutoKnockObjectOffDistanceInches = 18;
    public static final double AutoKnockObjectOffSpeed = .7;

    //public static final ArmPosition WITH_OBJECT_HOME_STATE = new ArmPosition(Position.WITH_OBJECT_HOME, 10, 6);
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
  // these are not in freedom units in meters :(
    @Override
    public double getDriveTrainVelocityConversion() {

      return 0.001388889;
    }

    @Override
    public double getDriveTrainPositionConversion() {
      return 1.7391;
    }

    @Override
    public double getAutoBalanceP() {
      return 0.032000;
    }

    @Override
    public double getAutoBalanceI() {
      return 0.001000;
    }

    @Override
    public double getAutoBalanceD() {
      return 0.002000;
    }
  }

  public static class TorinRobot implements IUniqueRobotConstants {

    @Override
    public String getName() {
      return "Torin";
    }
  // these are not in freedom units in meters :(
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

    @Override
    public double getAutoBalanceP() {
      return 0.032000;
    }

    @Override
    public double getAutoBalanceI() {
      return 0.001000;
    }

    @Override
    public double getAutoBalanceD() {
      return 0.002000;
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
    public double getAutoBalanceP();
    public double getAutoBalanceI();
    public double getAutoBalanceD();
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
