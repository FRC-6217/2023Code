package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TankDrive;

public class DriveToBalanced extends CommandBase {
  public static enum DirectionB{
    forwards, backwards
  }
  /** Creates a new Drivetounbalence. */
  TankDrive tankDrive;
  DirectionB direction;
  public DriveToBalanced(TankDrive tankDrive, DirectionB direction) {
    this.tankDrive = tankDrive;
    this.direction = direction;
    addRequirements(tankDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(direction == DirectionB.forwards)
    tankDrive.autoDrive(-.6, 0);
    else
    tankDrive.autoDrive(.6, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    tankDrive.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(this.tankDrive.getGyro().getPitch())<2;
  }
}
