// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.sensor.Gyro;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StayPut extends PIDCommand {

  /** Creates a new StayPut. */
    TankDrive mTankDrive;
  public StayPut(TankDrive mTankDrive, Gyro gyro) {
    super(
      new PIDController(0,0,0),
      // Close the loop on the turn rate
      gyro::getRoll,
      // Setpoint is 0
      0,
      // Pipe the output to the turning controls
      output -> mTankDrive.drive(output, 0),
      // Require the robot drive
        mTankDrive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    SmartDashboard.putData(this.getController());
    this.mTankDrive = mTankDrive;
  }

  @Override
  public void initialize() {
    super.initialize();
    this.getController().setSetpoint(mTankDrive.getRobotPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void execute() {
    super.execute();
    SmartDashboard.putData(this.getController());
    System.out.println("Hi:"+ this.getController().getP());

  }
}
