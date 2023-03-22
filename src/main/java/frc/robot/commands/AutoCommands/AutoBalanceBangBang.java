// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;


import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.TankDrive.RobotPosition;

public class AutoBalanceBangBang extends CommandBase {
  /** Creates a new AutoBalanceBangBang. */
  TankDrive tankDrive;
  WPI_Pigeon2 pigeon;
  RobotPosition sPosition;
  RobotPosition cPosition;

  double gyroRates[] = {0,0,0};

  double distanceToEstimatedBalancePointInches = 36;

  double maxSpeed = .7;
  double minSpeed = .3;
  double slope = (maxSpeed - minSpeed) / (distanceToEstimatedBalancePointInches - 0);

  double rateThreshold = 10; //dps
  double maxrate = 0;

  double direction = 1;

  Timer timer;

  public AutoBalanceBangBang(TankDrive tankDrive, double direction) {
    addRequirements(tankDrive);
    this.tankDrive = tankDrive;
    this.pigeon = tankDrive.getGyro();
    this.direction = direction;
    timer = new Timer();
  }

  
  public AutoBalanceBangBang(TankDrive tankDrive) {
    addRequirements(tankDrive);
    this.tankDrive = tankDrive;
    this.pigeon = tankDrive.getGyro();
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sPosition = tankDrive.getStartPosition();
    gyroRates = new double[] {0,0,0};
    timer.reset();
    timer.start();
    maxrate = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pigeon.getRawGyro(gyroRates);
    if (gyroRates[0] > maxrate) {
      maxrate = gyroRates[0];
      System.out.println("max: " + maxrate);
    }
    cPosition = tankDrive.getRelativePosition(sPosition);

    //is this inches?
   // tankDrive.autoDrive(direction*slope*cPosition.averagePosition + minSpeed, 0);
   tankDrive.autoDrive(direction*.28, 0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tankDrive.autoDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // is this pitch?
    return (timer.get() > .8) && (Math.abs(gyroRates[0]) > rateThreshold);// || (cPosition.averagePosition > distanceToEstimatedBalancePointInches);
  }
}
