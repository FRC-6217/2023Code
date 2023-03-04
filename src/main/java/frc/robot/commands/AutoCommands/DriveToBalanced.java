package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.TankDrive.RobotPosition;

public class DriveToBalanced extends CommandBase {
  public static enum DirectionB{
    forwards, backwards
  }
  /** Creates a new Drivetounbalence. */
  TankDrive tankDrive;
  DirectionB direction;
  RobotPosition sPosition;
  double speed;
  public DriveToBalanced(TankDrive tankDrive, DirectionB direction) {
    this.tankDrive = tankDrive;
    this.direction = direction;
    addRequirements(tankDrive);
    // Use addRequirements() here to declare subsystem dependencies.
    speed = .5;
  }

  public DriveToBalanced(TankDrive tankDrive, DirectionB direction, double speed){
    this(tankDrive, direction);
    this.speed = speed;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sPosition = tankDrive.getStartPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(direction == DirectionB.forwards)
    tankDrive.autoDrive(-speed, 0);
    else
    tankDrive.autoDrive(speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tankDrive.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isTooFar = Math.abs(tankDrive.getRelativePosition(sPosition).averagePosition) > 60;
    return Math.abs(this.tankDrive.getGyro().getPitch()) < 3 || isTooFar;
  }
}
