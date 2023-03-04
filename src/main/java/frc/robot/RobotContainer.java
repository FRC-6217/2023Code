// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.ArmSystemConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PneumaticConstants;
import frc.robot.Constants.ArmSystemConstants.LittleArmAngle;
import frc.robot.commands.CancelDriveTrain;
import frc.robot.commands.DriveToObject;
import frc.robot.commands.FindKs;
import frc.robot.commands.FindKv;
import frc.robot.commands.PersistenceData;
import frc.robot.commands.SlewRatedArmMovement;
import frc.robot.commands.StayPutAllDOF;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.ArmCommands.ArmGoToAngle;
import frc.robot.commands.ArmCommands.BigArmToLimitSwitch;
import frc.robot.commands.ArmCommands.LittleArmToLimitSwitch;
import frc.robot.commands.ArmCommands.SetArmZero;
import frc.robot.commands.ArmCommands.ToggldeClaw;
import frc.robot.commands.AutoCommands.AutoBalance;
import frc.robot.commands.AutoCommands.AutoBalancedPID;
import frc.robot.commands.AutoCommands.DriveToBalanced;
import frc.robot.commands.AutoCommands.DriveToDistanceInches;
import frc.robot.commands.AutoCommands.DriveUntilUnBalanced;
import frc.robot.commands.AutoCommands.EnableBrakes;
import frc.robot.commands.AutoCommands.DriveToBalanced.DirectionB;
import frc.robot.commands.AutoCommands.DriveUntilUnBalanced.Direction;
import frc.robot.commands.DriveToObject.ObjectType;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.PDP;
import frc.robot.subsystems.PIDDriveTrain;
import frc.robot.subsystems.PneumaticController;
import frc.robot.subsystems.PotentiameterTest;
import frc.robot.subsystems.ServoTEST;
import frc.robot.subsystems.SimpleMotorController;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.ArmSystem.ARM_SELECTION;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final PersistenceData mData = new PersistenceData();
  public final TankDrive mTankDrive = new TankDrive();
  public SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  //public final PneumaticController pneumatics = new PneumaticController();
  
  //private final InchesDrive inchesDrive12forward = new InchesDrive(mTankDrive, 12, .3);
 // private final InchesDrive inchesDrive12back = new InchesDrive(mTankDrive, -12, .3);

  final ArmSystem armSystem = new ArmSystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kXboxDriver);
  private final CommandJoystick driveJoystick = new CommandJoystick(OperatorConstants.kDriverControllerPort);
  private final CommandJoystick buttonBox = new CommandJoystick(OperatorConstants.buttonBoxPort);
  private final PDP pdp = new PDP();

  StayPutAllDOF stayPutCommand = new StayPutAllDOF(mTankDrive);
  CancelDriveTrain cancelCommand = new CancelDriveTrain(mTankDrive);
  //private final TestCoolBeans t = new TestCoolBeans();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putNumber("BigAutoArmAngle", 0);
    SmartDashboard.putNumber("LittleAutoArmAngle", 0);
    System.out.println("RUNNING ROBOT: " + Constants.uniqueRobotConstants.getName());
    // Configure the trigger bindings
    configureBindings();
    SmartDashboard.putData(autoChooser);
  
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //driveJoystick.button(1).onTrue(new FindKv(mTankDrive));
   // driveJoystick.button(1).onTrue(new SequentialCommandGroup(new InchesDrive(mTankDrive, 12, 0.3), new InchesDrive(mTankDrive, -12, 0.3)));
   // driveJoystick.button(2).onTrue(new GoToAngle(mTankDrive, -360, .36));

    AutoBalancedPID autoBalanceCommand = new AutoBalancedPID(mTankDrive);
    AutoBalancedPID autoBalanceCommandSeperate = new AutoBalancedPID(mTankDrive);

   // DriveUntilUnBalanced driveToChargingStation = new DriveUntilUnBalanced(mTankDrive, Direction.forwards);

    driveJoystick.button(OperatorConstants.toggleTurning12).onTrue(Commands.runOnce(mTankDrive::toggleTurning, mTankDrive));
    //driveJoystick.button(OperatorConstants.toggleBreak2).onTrue(Commands.runOnce(mTankDrive::toggleBreaks, mTankDrive));
    //driveJoystick.button(OperatorConstants.cancelDrive11).onTrue(cancelCommand);
    driveJoystick.button(OperatorConstants.doAutoBalance10).whileTrue(autoBalanceCommandSeperate);
   // driveJoystick.button(12).whileTrue(getMiddleLeaveBalance());
   // driveJoystick.button(OperatorConstants.fullBalanceAct6).onTrue(driveToChargingStation.andThen(autoBalanceCommand));
    //driveJoystick.button(OperatorConstants.buttonUnused7).onTrue(new DriveToDistanceInches(mTankDrive, 10, .4));
    //driveJoystick.button(OperatorConstants.buttonUnused9).whileTrue(getMiddleLeaveBalance());
   // driveJoystick.button(OperatorConstants.buttonUnused7).onTrue(new DriveToObject(mTankDrive, ObjectType.CONE));
   // driveJoystick.button(OperatorConstants.buttonUnused9).onTrue(new DriveToObject(mTankDrive, ObjectType.CUBE));
    //driveJoystick.button(OperatorConstants.stayPut11).toggleOnTrue(stayPutCommand);

//driveJoystick.button(4).onTrue(Commands.runOnce(armSystem::bigArmForward, armSystem)).onFalse(Commands.runOnce(armSystem::bigArmOff, armSystem));
//driveJoystick.button(6).onTrue(Commands.runOnce(armSystem::bigArmBackward)).onFalse(Commands.runOnce(armSystem::bigArmOff, armSystem));
//driveJoystick.button(5).onTrue(Commands.runOnce(armSystem::littleArmForward, armSystem)).onFalse(Commands.runOnce(armSystem::littleArmOff, armSystem));
//driveJoystick.button(3).onTrue(Commands.runOnce(armSystem::littleArmBackward, armSystem)).onFalse(Commands.runOnce(armSystem::littleArmOff, armSystem));
    driveJoystick.button(4).whileTrue(new ArmGoToAngle(armSystem, ARM_SELECTION.LITTLE_ARM, 0, true));

    //button box

    buttonBox.button(1).onTrue(Commands.runOnce(armSystem::toggleClaw, armSystem));
    //buttonBox.button(6).onTrue(Commands.runOnce(armSystem::littleArmForward, armSystem)).onFalse(Commands.runOnce(armSystem::littleArmOff, armSystem));
    // buttonBox.button(4).onTrue(Commands.runOnce(armSystem::littleArmBackward, armSystem)).onFalse(Commands.runOnce(armSystem::littleArmOff, armSystem));
    // buttonBox.button(5).onTrue(Commands.runOnce(armSystem::bigArmForward, armSystem)).onFalse(Commands.runOnce(armSystem::bigArmOff, armSystem));
    // buttonBox.button(3).onTrue(Commands.runOnce(armSystem::bigArmBackward)).onFalse(Commands.runOnce(armSystem::bigArmOff, armSystem));
    // //buttonBox.button(6).onTrue(Commands.runOnce(armSystem::enableBigArmBreak, armSystem));
    //buttonBox.button(7).onTrue(Commands.runOnce(armSystem::disableBigArmBreak, armSystem));
    buttonBox.button(6).whileTrue(new SlewRatedArmMovement(armSystem, ARM_SELECTION.LITTLE_ARM, 1));
    buttonBox.button(4).whileTrue(new SlewRatedArmMovement(armSystem, ARM_SELECTION.LITTLE_ARM, -1));
    buttonBox.button(5).whileTrue(new SlewRatedArmMovement(armSystem, ARM_SELECTION.BIG_ARM, 1));
    buttonBox.button(3).whileTrue(new SlewRatedArmMovement(armSystem, ARM_SELECTION.BIG_ARM, -1));
    buttonBox.button(10).whileTrue(new BigArmToLimitSwitch(armSystem));
    //buttonBox.button(8).whileTrue(getDummyAuto());
    //REMOVE For sure
    //buttonBox.button(7).whileTrue(extendToDrop());
    //buttonBox.button(8).whileTrue(retractWhileDriving());
    //buttonBox.button(9).whileTrue(zeroArms());
    //todo set buttons
   // buttonBox.button(7).whileTrue(new ArmGoToAngle(armSystem, ARM_SELECTION.BIG_ARM, 0, true));
    //buttonBox.button(0).onTrue(new ArmGoToAngle(armSystem, ARM_SELECTION.LITTLE_ARM, 0 , true));


    //auto

    mTankDrive.setDefaultCommand( new TeleopDrive(mTankDrive, driveJoystick));
    autoChooser.setDefaultOption("Left Leave", getLeftAuto());
    autoChooser.addOption("Right Leave", getRightAuto(.5));
    autoChooser.addOption("Middle Leave Balance", getMiddleLeaveBalance());
    autoChooser.addOption("Do Nothing", getDoNothingCommand());
    //autoChooser.addOption("1BallAndLeave", autoDropObject());
    //autoChooser.addOption("NoPIDMiddle", getNoPIDMiddleLeaveBalance());
    autoChooser.addOption("Low Drop Off Back Up", getLowDropOffBackUp());
    autoChooser.addOption("high drop off", getAutoDropOffHigh());
    autoChooser.addOption("HighDropWithBalance", AutoHighDropAndBalance());
//    autoChooser.addOption("HighDropWithBalanceOverandBack", AutoHighDropAndBalanceOver());



    //driveJoystick.button(OperatorConstants.resetRobotPosition4).onTrue(Commands.runOnce(mTankDrive::resetPosition, mTankDrive));
    //driveJoystick.button(6).onTrue(new Drivetounbalence(mTankDrive).andThen(new AngleDrive(mTankDrive)));

    //driveJoystick.button(OperatorConstants.littleArmFoward).onTrue(Commands.runOnce(littleArm::on, littleArm)).onFalse(Commands.runOnce(littleArm::off, littleArm));
    //driveJoystick.button(OperatorConstants.bigArmForward).onTrue(Commands.runOnce(bigArm::on, bigArm)).onFalse(Commands.runOnce(bigArm::off, bigArm));
    //driveJoystick.button(OperatorConstants.littleArmBack).onTrue(Commands.runOnce(littleArm::reverse, littleArm)).onFalse(Commands.runOnce(littleArm::off, littleArm));
    //driveJoystick.button(OperatorConstants.bigArmBack).onTrue(Commands.runOnce(bigArm::reverse, bigArm)).onFalse(Commands.runOnce(bigArm::off, bigArm));
    //CommandScheduler.getInstance().setDefaultCommand(mTankDrive, new TeleopDrive(mTankDrive, driveJoystick));



    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public SequentialCommandGroup getMiddleLeaveBalance(){
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();
    commandGroup.addCommands(new EnableBrakes(mTankDrive));
    commandGroup.addCommands(new PrintCommand("Driving off1"));
    commandGroup.addCommands(new DriveUntilUnBalanced(mTankDrive, Direction.backwards));
    commandGroup.addCommands(new PrintCommand("Driving off2"));
    commandGroup.addCommands(new DriveToDistanceInches(mTankDrive, AutoConstants.middleLeaveOnPlatformInches, AutoConstants.middleLeaveOffPlatformSpeed));
    commandGroup.addCommands(new PrintCommand("Driving off3"));
    commandGroup.addCommands(new DriveToBalanced(mTankDrive, DirectionB.forwards));
    commandGroup.addCommands(new PrintCommand("Driving off4"));
    commandGroup.addCommands(new DriveToDistanceInches(mTankDrive, AutoConstants.middleLeaveOffPlatformInches, AutoConstants.middleLeaveOffPlatformSpeed));
    commandGroup.addCommands(new PrintCommand("Driving off5"));
    commandGroup.addCommands(new DriveUntilUnBalanced(mTankDrive, Direction.forwards));
    commandGroup.addCommands(new PrintCommand("Driving off6"));
    commandGroup.addCommands(new AutoBalancedPID(mTankDrive));
    return commandGroup;
  }
  public SequentialCommandGroup getNoPIDMiddleLeaveBalance(){
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();
    commandGroup.addCommands(new EnableBrakes(mTankDrive));
    commandGroup.addCommands(new PrintCommand("Driving off1"));
    commandGroup.addCommands(new DriveUntilUnBalanced(mTankDrive, Direction.backwards));
    commandGroup.addCommands(new PrintCommand("Driving off2"));
    commandGroup.addCommands(new DriveToDistanceInches(mTankDrive, AutoConstants.middleLeaveOnPlatformInches, AutoConstants.middleLeaveOffPlatformSpeed));
    commandGroup.addCommands(new PrintCommand("Driving off3"));
    commandGroup.addCommands(new DriveToBalanced(mTankDrive, DirectionB.forwards)); //Make sure on the floor
    commandGroup.addCommands(new PrintCommand("Driving off4"));
    commandGroup.addCommands(new DriveToDistanceInches(mTankDrive, AutoConstants.middleLeaveOffPlatformInches, AutoConstants.middleLeaveOffPlatformSpeed));
    commandGroup.addCommands(new PrintCommand("Driving off5"));
    commandGroup.addCommands(new DriveUntilUnBalanced(mTankDrive, Direction.forwards));
    commandGroup.addCommands(new PrintCommand("Driving off6"));
    commandGroup.addCommands(new DriveToBalanced(mTankDrive, DirectionB.backwards, .3));
    return commandGroup;
  }


  public SequentialCommandGroup getLeftAuto() {
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();
    //commandGroup.addCommands(Commands.runOnce(armSystem::toggleClaw, armSystem));
    commandGroup.addCommands(new EnableBrakes(mTankDrive));
    commandGroup.addCommands(Commands.waitSeconds(.5));
    commandGroup.addCommands(new DriveToDistanceInches(mTankDrive, AutoConstants.leftLeaveDistanceInches, AutoConstants.leftLeaveSpeed));
    return commandGroup;
  }

  public SequentialCommandGroup getRightAuto(double secondsToWait) {
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();
    //commandGroup.addCommands(Commands.runOnce(armSystem::toggleClaw, armSystem));
    commandGroup.addCommands(new EnableBrakes(mTankDrive));
    commandGroup.addCommands(Commands.waitSeconds(secondsToWait));
    commandGroup.addCommands(new DriveToDistanceInches(mTankDrive, AutoConstants.rightLeaveDistanceInches, AutoConstants.rightLeaveSpeed));
    return commandGroup;
  }

  public SequentialCommandGroup getHighDropAndLeave(){
    return null;
  }

  public SequentialCommandGroup getLowDropOffBackUp(){
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();
    commandGroup.addCommands(Commands.waitSeconds(.5));
    commandGroup.addCommands(getAutoDropOffLow());
    commandGroup.addCommands(getRightAuto(0.1));

    return commandGroup;
  }

  public SequentialCommandGroup getAutoDropOffLow() {
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();
    commandGroup.addCommands(new ArmGoToAngle(armSystem, ARM_SELECTION.LITTLE_ARM, -45));
    commandGroup.addCommands(Commands.runOnce(armSystem::toggleClaw, armSystem));
    commandGroup.addCommands(Commands.waitSeconds(.5));
    commandGroup.addCommands(new ArmGoToAngle(armSystem, ARM_SELECTION.LITTLE_ARM, 0));

    return commandGroup;
  }

  public SequentialCommandGroup getAutoDropOffMiddle(){
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();
    commandGroup.addCommands(new ArmGoToAngle(armSystem, ARM_SELECTION.LITTLE_ARM, -90));
    commandGroup.addCommands(new DriveToDistanceInches(mTankDrive, 12, .3));
    commandGroup.addCommands(Commands.runOnce(armSystem::toggleClaw, armSystem));
    commandGroup.addCommands(Commands.waitSeconds(.5));
    commandGroup.addCommands(new DriveToDistanceInches(mTankDrive, -12, 0.3));
    commandGroup.addCommands(new ArmGoToAngle(armSystem, ARM_SELECTION.LITTLE_ARM, 0));
  
    return commandGroup;
  }

  public SequentialCommandGroup getAutoDropOffHigh(){
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();
    commandGroup.addCommands(Commands.waitSeconds(.5));
    commandGroup.addCommands(new ArmGoToAngle(armSystem, ARM_SELECTION.LITTLE_ARM, -40));
    commandGroup.addCommands(new BigArmToLimitSwitch(armSystem));
    commandGroup.addCommands(new ArmGoToAngle(armSystem, ARM_SELECTION.LITTLE_ARM, -157));
    commandGroup.addCommands(new DriveToDistanceInches(mTankDrive, 24, .45));
    commandGroup.addCommands(Commands.runOnce(armSystem::toggleClaw, armSystem));
    commandGroup.addCommands(Commands.waitSeconds(.5));
    commandGroup.addCommands(new DriveToDistanceInches(mTankDrive, -24, 0.45));
    commandGroup.addCommands(new ArmGoToAngle(armSystem, ARM_SELECTION.BIG_ARM, 10));
    commandGroup.addCommands(new LittleArmToLimitSwitch(armSystem));
    System.out.println("Done With Little");
  
    return commandGroup;
  }

  
  public Command getDoNothingCommand() {
    return new EnableBrakes(mTankDrive);
  }


  public SequentialCommandGroup zeroArms() {

    SequentialCommandGroup commandGroup = new SequentialCommandGroup();

    // Move Little and big arm
    commandGroup.addCommands(new SetArmZero(armSystem, ARM_SELECTION.BIG_ARM));
    commandGroup.addCommands(new SetArmZero(armSystem, ARM_SELECTION.LITTLE_ARM));

    return commandGroup;
  }

  public SequentialCommandGroup extendToDrop() {
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();

    // Extend little and big arm
    commandGroup.addCommands(new ArmGoToAngle(armSystem, ARM_SELECTION.BIG_ARM, SmartDashboard.getNumber("BigAutoArmAngle", 0)));
    commandGroup.addCommands(new ArmGoToAngle(armSystem, ARM_SELECTION.LITTLE_ARM, SmartDashboard.getNumber("LittleAutoArmAngle", 0)));
    return commandGroup; 
  }

  public ParallelCommandGroup retractWhileDriving(){
    ParallelCommandGroup parallelCommandGroup = new ParallelCommandGroup();
    parallelCommandGroup.addCommands(new DriveToDistanceInches(mTankDrive, Constants.AutoConstants.leftLeaveDistanceInches, Constants.AutoConstants.leftLeaveSpeed));
    parallelCommandGroup.addCommands(zeroArms());

    return parallelCommandGroup;
  }

  public SequentialCommandGroup AutoHighDropAndBalance(){
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();
    commandGroup.addCommands(new EnableBrakes(mTankDrive));
    commandGroup.addCommands(getAutoDropOffHigh());
    commandGroup.addCommands(new PrintCommand("Driving off1"));
    commandGroup.addCommands(new DriveUntilUnBalanced(mTankDrive, Direction.backwards));
    commandGroup.addCommands(new DriveToDistanceInches(mTankDrive, -18, .7));
    commandGroup.addCommands(new PrintCommand("Driving off3"));
    commandGroup.addCommands(new AutoBalancedPID(mTankDrive));
    return commandGroup;

  }

 /* public SequentialCommandGroup AutoHighDropAndBalanceOver(){
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();
    commandGroup.addCommands(new EnableBrakes(mTankDrive));
    commandGroup.addCommands(getAutoDropOffHigh());
    commandGroup.addCommands(getMiddleLeaveBalance());
    return commandGroup;

  }*/
  public SequentialCommandGroup autoDropObject(){
    SequentialCommandGroup commandGroup = new SequentialCommandGroup();
    commandGroup.addCommands(new DriveToDistanceInches(mTankDrive, -20, 0.4));
    commandGroup.addCommands(Commands.runOnce(armSystem::toggleClaw, armSystem));
    commandGroup.addCommands(zeroArms());
    commandGroup.addCommands(extendToDrop());
    commandGroup.addCommands(new DriveToDistanceInches(mTankDrive, 20, 0.4));
    commandGroup.addCommands(Commands.runOnce(armSystem::toggleClaw, armSystem));
    commandGroup.addCommands(new DriveToDistanceInches(mTankDrive, -20, 0.4));
    commandGroup.addCommands(retractWhileDriving());

    return commandGroup;
  }

  public ParallelCommandGroup retractAfterDropOff() {

    ParallelCommandGroup commandGroup = new ParallelCommandGroup();

    // Retract little and big arm
    commandGroup.addCommands();

    return commandGroup;
  }
}