// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.SwerveSubsystems.*;
import frc.robot.Autonomous.ApproachTagCommand;
import frc.robot.Autonomous.CenterOnTagCommand;
import frc.robot.Autonomous.DeltaPoseCommand;
import frc.robot.Autonomous.LookForTagCommand;
import frc.robot.Autonomous.MoveToTagCommand;
import frc.robot.Autonomous.PullClimber;
import frc.robot.Autonomous.HeightBasedElevatorCommand;
import frc.robot.Commands.*;
import frc.robot.Information.*;
import frc.robot.OtherSubsystems.ClimberSubsystem;
import frc.robot.OtherSubsystems.TowerSubsystem;

public class RobotContainer {
  public RobotContainer() {
    
  }

  // Base inits
            // Controllers
            // Joystick hotaz = new Joystick(0);
            Joystick towerJoy = new Joystick(1);
            XboxController xbox = new XboxController(0);
          
            // Subsystems
            DriveSubsystem driveSub = new DriveSubsystem();
            OdometrySubsystem odomSub = new OdometrySubsystem(driveSub);
            TagSubsystem tagSub = new TagSubsystem(odomSub);
            TowerSubsystem towerSub = new TowerSubsystem();
            ClimberSubsystem climbSub = new ClimberSubsystem();
            
            // Commands from files
            // HOTASDriveCommand HOTASDriveCommand = new HOTASDriveCommand(driveSub, hotaz, tagSub, odomSub);
            XBOXDriveCommand xboxDriveCommand = new XBOXDriveCommand(driveSub, xbox, tagSub, odomSub, towerSub);
            TowerControlCommand towerCommand = new TowerControlCommand(towerSub, xbox, towerJoy);
            ClimberControlCommand climberCommand = new ClimberControlCommand(climbSub, towerJoy);
            LookForTagCommand lookForTagCommand = new LookForTagCommand(tagSub);
            HeightBasedElevatorCommand moveToMaxHeight = new HeightBasedElevatorCommand(1, towerSub);
            
            //Command scheduler
            {
              driveSub.setDefaultCommand(xboxDriveCommand);
              climbSub.setDefaultCommand(climberCommand);
              towerSub.setDefaultCommand(towerCommand);
              tagSub.setDefaultCommand(lookForTagCommand);
              Trigger lT = new Trigger(() -> {return xbox.getLeftTriggerAxis() > 0.85;});
              Trigger rT = new Trigger(() -> {return xbox.getRightTriggerAxis() > 0.85;});
              Trigger lB = new Trigger(() -> {return xbox.getLeftBumperButtonPressed();});
              Trigger rB = new Trigger(() -> {return xbox.getRightBumperButtonPressed();});
              lT.onTrue(new MoveToTagCommand(0.3, 0, driveSub, odomSub, tagSub, xbox));
              rT.onTrue(new MoveToTagCommand(0.3, -0.32, driveSub, odomSub, tagSub, xbox));
              lB.onTrue(new CenterOnTagCommand(0.16, driveSub, odomSub, tagSub, xbox));
              rB.onTrue(new CenterOnTagCommand(-0.16, driveSub, odomSub, tagSub, xbox));
            }

  ApproachTagCommand tagCommand = new ApproachTagCommand(tagSub, driveSub);

  //Autonomous chooser
  public Command getAutonomousCommand(String autoChosen) {
    // switch (autoChosen){
    //   case "Auto 1": return new ParallelCommandGroup(new PullClimber(climbSub, 0.05), new WaitCommand(5)).andThen(
    //     new DeltaPoseCommand(0, 1.5, 0, driveSub, odomSub));
    //   case "Auto 2": return new DeltaPoseCommand(0, 1.5, 0, driveSub, odomSub);
    //   case "Auto 3": return new MoveToTagCommand(0.3, driveSub, odomSub, tagSub);
    //   case "Auto 4": return doATwirl;
    //   case "Auto 5": return doADance;
    //   case "Auto 6": return heightLadder;
    //   default: return null;
    // }
    // return new ParallelCommandGroup(new PullClimber(climbSub, 0.05), new WaitCommand(5)).andThen(
    //   new DeltaPoseCommand(0, 1.5, 0, driveSub, odomSub));
    // return new MoveToTagCommand(0.2, 0.32, driveSub, odomSub, tagSub);
    return new DeltaPoseCommand(0, 1.5, 0, driveSub, odomSub);
  }
  
  








  //From here on out is autonomous hell :)



  public ClimberSubsystem returnClimber() {
    return climbSub;
  }









  
  SequentialCommandGroup heightLadder = new HeightBasedElevatorCommand(0.5, towerSub).andThen(
    new HeightBasedElevatorCommand(0.25, towerSub).andThen(
      new HeightBasedElevatorCommand(0.75, towerSub).andThen(
        new HeightBasedElevatorCommand(0, towerSub)
      )
    )
  );
  
  SequentialCommandGroup doADance = new ParallelCommandGroup(
    new DeltaPoseCommand(0.5, 0.5, -Math.PI/2, driveSub, odomSub),
    new HeightBasedElevatorCommand(0.25, towerSub)
  ).andThen(new ParallelCommandGroup(
    new DeltaPoseCommand(-1, 1, Math.PI/2, driveSub, odomSub),
    new HeightBasedElevatorCommand(0.5, towerSub)
  )).andThen(new ParallelCommandGroup(
    new DeltaPoseCommand(1, 1, -Math.PI/2, driveSub, odomSub),
    new HeightBasedElevatorCommand(0.75, towerSub)
  )).andThen(new ParallelCommandGroup(
    new DeltaPoseCommand(-1, 1, Math.PI/2, driveSub, odomSub),
    new HeightBasedElevatorCommand(1, towerSub)
  )).andThen(new ParallelCommandGroup(
    new DeltaPoseCommand(0.5, -3.5, 0, driveSub, odomSub),
    new HeightBasedElevatorCommand(0, towerSub)
  ));
  
  SequentialCommandGroup doATwirl = new ParallelCommandGroup(
    new DeltaPoseCommand(0, 1, -Math.PI, driveSub, odomSub),
    new HeightBasedElevatorCommand(0.25, towerSub)
  ).andThen(new ParallelCommandGroup(
    new DeltaPoseCommand(-1, -1, Math.PI, driveSub, odomSub),
    new HeightBasedElevatorCommand(0.5, towerSub)
  )).andThen(new ParallelCommandGroup(
    new DeltaPoseCommand(1, -1, Math.PI, driveSub, odomSub),
    new HeightBasedElevatorCommand(0.5, towerSub)
  )).andThen(new ParallelCommandGroup(
    new DeltaPoseCommand(1, 1, 0, driveSub, odomSub),
    new HeightBasedElevatorCommand(0.5, towerSub)
  )).andThen(new ParallelCommandGroup(
    new DeltaPoseCommand(-1, -1, Math.PI/2, driveSub, odomSub),
    new HeightBasedElevatorCommand(0.5, towerSub)
  )).andThen(new ParallelCommandGroup(
    new DeltaPoseCommand(-1, 1, Math.PI, driveSub, odomSub),
    new HeightBasedElevatorCommand(0.5, towerSub)
  )).andThen(new ParallelCommandGroup(
    new DeltaPoseCommand(1, 1, -Math.PI/2, driveSub, odomSub),
    new HeightBasedElevatorCommand(0.5, towerSub)
  )).andThen(new ParallelCommandGroup(
    new DeltaPoseCommand(0, -1, 0, driveSub, odomSub),
    new HeightBasedElevatorCommand(0.5, towerSub)
  ));
}

