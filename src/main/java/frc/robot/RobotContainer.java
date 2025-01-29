// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.SwerveSubsystems.*;
import frc.robot.Autonomous.RotateToCommand;
import frc.robot.Commands.*;
import frc.robot.Information.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }
  //Auto chooser for Robot.java
  // Controllers
  Joystick hotas = new Joystick(0);
  
  XboxController secondController = new XboxController(1);
  
  // Subsystems
  NavigationSubsystem navSub;
  DriveSubsystem driveSub = new DriveSubsystem(navSub);
  navSub = new NavigationSubsystem(driveSub);
  TagSubsystem tagSubsystem = new TagSubsystem(navSub);
  
  // Commands from files
  DriveCommand driveCommand = new DriveCommand(driveSub, hotas, secondController, navSub);
  PDownCommand PDownCommand = new PDownCommand(driveSub);
  
  RotateToCommand rotateToCommand = new RotateToCommand(Math.PI/2, driveSub);
  //Buttons
  //driveStick
  // JoystickButton resetMotorsButton = new JoystickButton(driveStick, 4);
  // JoystickButton resetOrientationButton = new JoystickButton(hotas, 7);
  
  
    
  
  private void configureBindings() {
    driveSub.setDefaultCommand(driveCommand);
  }
  public Command getAutonomousCommand() {
    return rotateToCommand;
  }
}
