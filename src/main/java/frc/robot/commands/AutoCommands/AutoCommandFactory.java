// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.ArmCommands.ArmPositions;
import frc.robot.commands.AutoCommands.DriveToBalanced.DirectionB;
import frc.robot.commands.AutoCommands.DriveUntilUnBalanced.Direction;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.ArmSystem.Arm;
import frc.robot.subsystems.ArmSystem.BigArm;
import frc.robot.subsystems.ArmSystem.Claw;

/** Add your docs here. */
public class AutoCommandFactory {
    Arm littleArm;
    BigArm bigArm;
    TankDrive tankDrive;
    ArmPositions armPositions;
    Claw claw;
    //Potenial Maybe Use
    // public SendableChooser<Command> autoChooserStep1 = new SendableChooser<Command>();
    // public SendableChooser<Command> autoChooserStep2 = new SendableChooser<Command>();
    public SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    SequentialCommandGroup testCommand;

    public AutoCommandFactory(TankDrive tankDrive, Arm littArm, BigArm bigArm, Claw claw){
        this.bigArm = bigArm;
        this.littleArm = littArm;
        this.tankDrive = tankDrive;
        this.claw = claw;
        this.armPositions = new ArmPositions(littArm, bigArm);

        testCommand = new SequentialCommandGroup();

        autoChooser.setDefaultOption("No Drop off, no move", AlwaysDo().andThen(armPositions.ArmsToSaftey()));

        autoChooser.addOption("High Cube Drop Off, no move", AlwaysDo().andThen(doDropOffAtHighCube()));
        autoChooser.addOption("Mid Cube Drop Off, no move", AlwaysDo().andThen(doDropOffAtMidCube()));
        autoChooser.addOption("High Cone Drop Off, no move", AlwaysDo().andThen(doDropOffAtHighCone()));


        autoChooser.addOption("High Cube Drop Off, then move out of community", AlwaysDo().andThen(doDropOffAtHighCube()).andThen(getLeaveAuto()));
        autoChooser.addOption("Mid Cube Drop Off, then move out of community", AlwaysDo().andThen(doDropOffAtHighCube()).andThen(getLeaveAuto()));
        autoChooser.addOption("High Cone Drop Off, then move out of community", AlwaysDo().andThen(doDropOffAtHighCone()).andThen(getLeaveAuto()));
        autoChooser.addOption("No Drop off, then move out of community", AlwaysDo().andThen(getLeaveAuto()));

        autoChooser.addOption("High Cube Drop Off, then balance", AlwaysDo().andThen(doDropOffAtHighCube()).andThen(NoLeaveBalanceArmsToSaftey()));
        autoChooser.addOption("Mid Cube Drop Off, then balance", AlwaysDo().andThen(doDropOffAtMidCube()).andThen(NoLeaveBalanceArmsToSaftey()));
        autoChooser.addOption("High Cone Drop Off, then balance", AlwaysDo().andThen(doDropOffAtHighCone()).andThen(NoLeaveBalanceArmsToSaftey()));
        autoChooser.addOption("No Drop off, then balance", AlwaysDo().andThen(NoLeaveBalanceArmsToSaftey()));


        autoChooser.addOption("High Cube Drop Off, then leave and balance", AlwaysDo().andThen(doDropOffAtHighCube()).andThen(DriveOverChargeStationArmsToSaftey()));
        autoChooser.addOption("Mid Cube Drop Off, then leave and balance", AlwaysDo().andThen(doDropOffAtMidCube()).andThen(DriveOverChargeStationArmsToSaftey()));
        autoChooser.addOption("High Cone Drop Off, then leave and balance", AlwaysDo().andThen(doDropOffAtHighCone()).andThen(DriveOverChargeStationArmsToSaftey()));
        autoChooser.addOption("No Drop off, then leave and balance", AlwaysDo().andThen(DriveOverChargeStationArmsToSaftey()));
        autoChooser.addOption("High Cube Drop Off V2, then leave and balance", AlwaysDo().andThen(doDropOffAtHighCube()).andThen(DriveOverChargeStationArmsToSafteyV2()));

        
/*
        autoChooserStep1.setDefaultOption("No Drop Off", armPositions.ArmsToSaftey());
        autoChooserStep1.addOption("Mid Cube Drop Off", doDropOffAtMidCube());
        autoChooserStep1.addOption("High Cube Drop Off", doDropOffAtHighCube());
        autoChooserStep1.addOption("Mid Cone Drop Off", doDropOffAtMidCone());
        autoChooserStep1.addOption("High Cone Drop Off", doDropOffAtHighCone());
        autoChooserStep1.addOption("Low Drop Off", doDropOffAtLow());

        autoChooserStep2.setDefaultOption("Back Out Of Community", getLeaveAuto());
        autoChooserStep2.addOption("No Leave Balance", NoLeaveBalanceArmsToSaftey());
        autoChooserStep2.addOption("Leave and Balance", DriveOverChargeStationArmsToSaftey());
        autoChooserStep2.addOption("Don't Move", armPositions.ArmsToSaftey());

        SmartDashboard.putData(autoChooserStep2);
        SmartDashboard.putData(autoChooserStep1);
         */

         SmartDashboard.putData(autoChooser);
    }

    public SequentialCommandGroup doDropOffAtMidCube() {
        SequentialCommandGroup sCommandGroup = new SequentialCommandGroup();
        sCommandGroup.addCommands(armPositions.ArmsToMidCubeDrop());
        sCommandGroup.addCommands(new DriveToDistanceInches(tankDrive, Constants.AutoConstants.AutoDriveToHubInches, Constants.AutoConstants.AutoDriveStep1Speed));
        sCommandGroup.addCommands(Commands.runOnce(claw::open, claw));
        sCommandGroup.addCommands(Commands.waitSeconds(.3));
        sCommandGroup.addCommands(new DriveToDistanceInches(tankDrive, -Constants.AutoConstants.AutoDriveToHubInches, Constants.AutoConstants.AutoDriveStep1Speed));
        return sCommandGroup;
    }

    public SequentialCommandGroup doDropOffAtHighCube() {
        SequentialCommandGroup sCommandGroup = new SequentialCommandGroup();
        sCommandGroup.addCommands(armPositions.ArmsToHighCubeDrop());
        sCommandGroup.addCommands(new DriveToDistanceInches(tankDrive, Constants.AutoConstants.AutoDriveToHubInches, Constants.AutoConstants.AutoDriveStep1Speed));
        sCommandGroup.addCommands(Commands.runOnce(claw::open, claw));
        sCommandGroup.addCommands(Commands.waitSeconds(.3));
        sCommandGroup.addCommands(new DriveToDistanceInches(tankDrive, -Constants.AutoConstants.AutoDriveToHubInches, Constants.AutoConstants.AutoDriveStep1Speed));
        return sCommandGroup;
    }

    public SequentialCommandGroup doDropOffAtMidCone() {
        SequentialCommandGroup sCommandGroup=new SequentialCommandGroup();
        sCommandGroup.addCommands(armPositions.ArmsToMidConeDrop());
        sCommandGroup.addCommands(new DriveToDistanceInches(tankDrive, Constants.AutoConstants.AutoDriveToHubInches, Constants.AutoConstants.AutoDriveStep1Speed));
        sCommandGroup.addCommands(Commands.runOnce(claw::open, claw));
        sCommandGroup.addCommands(Commands.waitSeconds(.3));
        sCommandGroup.addCommands(new DriveToDistanceInches(tankDrive, -Constants.AutoConstants.AutoDriveToHubInches, Constants.AutoConstants.AutoDriveStep1Speed));
        return sCommandGroup;
    }

    public SequentialCommandGroup doDropOffAtHighCone() {
        SequentialCommandGroup sCommandGroup=new SequentialCommandGroup();
        sCommandGroup.addCommands(Commands.waitSeconds(.1));
        sCommandGroup.addCommands(armPositions.ArmsToHighConeDrop(3));
        sCommandGroup.addCommands(new DriveToDistanceInches(tankDrive, Constants.AutoConstants.AutoDriveToHubConeHighInches, Constants.AutoConstants.AutoDriveStep1Speed));
        sCommandGroup.addCommands(Commands.runOnce(claw::open, claw));
        sCommandGroup.addCommands(Commands.waitSeconds(.3));
        sCommandGroup.addCommands(new DriveToDistanceInches(tankDrive, -Constants.AutoConstants.AutoDriveToHubConeHighInches, Constants.AutoConstants.AutoDriveStep1Speed));
        return sCommandGroup;
    }

    public SequentialCommandGroup doDropOffAtLow() {
        SequentialCommandGroup sCommandGroup=new SequentialCommandGroup();
        sCommandGroup.addCommands(armPositions.ArmsToFrontPickUp());
        sCommandGroup.addCommands(new DriveToDistanceInches(tankDrive, Constants.AutoConstants.AutoDriveToHubInches, Constants.AutoConstants.AutoDriveStep1Speed));
        sCommandGroup.addCommands(Commands.runOnce(claw::open, claw));
        sCommandGroup.addCommands(Commands.waitSeconds(.2));
        sCommandGroup.addCommands(new DriveToDistanceInches(tankDrive, -Constants.AutoConstants.AutoDriveToHubInches, Constants.AutoConstants.AutoDriveStep1Speed));
        return sCommandGroup;
    }



    public /*SequentialCommandGroup*/ Command getAutoCommand(){

        //SequentialCommandGroup sCommandGroup = new SequentialCommandGroup();

        // sCommandGroup.addCommands(this.AlwaysDo());
        // sCommandGroup.addCommands(this.getStep1());
        // sCommandGroup.addCommands(this.getStep2());

        return autoChooser.getSelected();
    }
/*
    public Command getStep1() {
        return autoChooserStep1.getSelected();
    }

    public Command getStep2() {
        return autoChooserStep2.getSelected();
    }
*/
    public SequentialCommandGroup DriveOverChargeStation(){
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        commandGroup.addCommands(new DriveUntilUnBalanced(tankDrive, Direction.backwards));
        commandGroup.addCommands(new DriveToDistanceInches(tankDrive, AutoConstants.middleLeaveOnPlatformInches, AutoConstants.middleLeaveOffPlatformSpeed));
        commandGroup.addCommands(new DriveToBalanced(tankDrive, DirectionB.forwards, 0.4));
        commandGroup.addCommands(new DriveToDistanceInches(tankDrive, AutoConstants.middleLeaveOffPlatformInches, AutoConstants.middleLeaveOffPlatformSpeed));
        commandGroup.addCommands(new DriveUntilUnBalanced(tankDrive, Direction.forwards));
        commandGroup.addCommands(new DriveToDistanceInches(tankDrive, 34, 0.6));
        commandGroup.addCommands(new WaitCommand(.4));
        commandGroup.addCommands(new AutoBalancedPID(tankDrive));
        return commandGroup;
    }

    public SequentialCommandGroup DriveOverChargeStationV2() {
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        commandGroup.addCommands(new DriveUntilUnBalanced(tankDrive, Direction.backwards));
        commandGroup.addCommands(new DriveToDistanceInches(tankDrive, AutoConstants.middleLeaveOnPlatformInches, AutoConstants.middleLeaveOffPlatformSpeed));
        commandGroup.addCommands(new DriveToBalanced(tankDrive, DirectionB.forwards, 0.4));
        commandGroup.addCommands(new DriveToDistanceInches(tankDrive, AutoConstants.middleLeaveOffPlatformInches, AutoConstants.middleLeaveOffPlatformSpeed));
        commandGroup.addCommands(new DriveUntilUnBalanced(tankDrive, Direction.forwards).andThen(new DriveToDistanceInches(tankDrive, 26, .6))/*.andThen(new AutoBalanceBangBang(tankDrive))*/.andThen(new AutoBalancedPID(tankDrive)));
        return commandGroup;
    }

    public SequentialCommandGroup NoLeaveBalance(){
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        commandGroup.addCommands(new DriveUntilUnBalanced(tankDrive, Direction.backwards));
        commandGroup.addCommands(new DriveToDistanceInches(tankDrive, -34, AutoConstants.middleLeaveOffPlatformSpeed));
        commandGroup.addCommands(new AutoBalancedPID(tankDrive));
        return commandGroup;
    }

    public ParallelCommandGroup NoLeaveBalanceArmsToSaftey(){
        ParallelCommandGroup pCommandGroup = new ParallelCommandGroup();
        pCommandGroup.addCommands(NoLeaveBalance());
        pCommandGroup.addCommands(armPositions.ArmsToSaftey());
        return pCommandGroup;
    }


    public ParallelCommandGroup  DriveOverChargeStationArmsToSaftey(){
        ParallelCommandGroup pCommandGroup = new ParallelCommandGroup();
        pCommandGroup.addCommands(DriveOverChargeStation());
        pCommandGroup.addCommands(armPositions.ArmsToSaftey());
        return pCommandGroup;
    }

    public ParallelCommandGroup  DriveOverChargeStationArmsToSafteyV2(){
        ParallelCommandGroup pCommandGroup = new ParallelCommandGroup();
        pCommandGroup.addCommands(DriveOverChargeStationV2());
        pCommandGroup.addCommands(armPositions.ArmsToSaftey());
        return pCommandGroup;
    }

    public ParallelCommandGroup DriveOverChargeStationArmsFlipOver(){
        ParallelCommandGroup pCommandGroup = new ParallelCommandGroup();
        pCommandGroup.addCommands(DriveOverChargeStation());
        return pCommandGroup;
    }

    public ParallelCommandGroup getLeaveAuto() {
        ParallelCommandGroup commandGroup = new ParallelCommandGroup();
        commandGroup.addCommands(armPositions.ArmsToSaftey());
        commandGroup.addCommands(new DriveToDistanceInches(tankDrive, AutoConstants.leftLeaveDistanceInches, AutoConstants.leftLeaveSpeed));
        return commandGroup;
      }

    public SequentialCommandGroup AlwaysDo(){
        SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
        pCommandGroup.addCommands(Commands.runOnce(tankDrive.getGyro()::reset, tankDrive));
        pCommandGroup.addCommands(new EnableBrakes(tankDrive));
        pCommandGroup.addCommands(Commands.runOnce(claw::close, claw));
        pCommandGroup.addCommands(Commands.waitSeconds(.2));
        return pCommandGroup;
    }
}
