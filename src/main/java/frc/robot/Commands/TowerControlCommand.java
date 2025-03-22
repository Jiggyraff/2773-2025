// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Autonomous.MoveToTagCommand;
import frc.robot.OtherSubsystems.TowerSubsystem;

public class TowerControlCommand extends Command {
  TowerSubsystem towerSub;
  Joystick joy;
  
  private final XboxController xbox;
  boolean simple;

  /** Creates a new TowerControlCommand. */
  public TowerControlCommand(TowerSubsystem towerSub, XboxController xbox, Joystick joy) {
    addRequirements(towerSub);
    this.towerSub = towerSub;
    this.joy = joy;
    
    this.xbox = xbox;
  }

  @Override
  public void initialize() {
    towerSub.runElevatorMotors(0);
  }
  
  @Override
  public void execute() {

    //#######       #######
    //####### HOTAS #######
    //#######       #######

    //Move elevator HOTAS
    if (!simple) {
      towerSub.setDifferenceHeight(MathUtil.applyDeadband(joy.getY(), 0.03) * Constants.MaxTowerSpeed);
    } else {
      towerSub.runElevatorMotors(MathUtil.applyDeadband(joy.getY(), 0.03) * Constants.MaxTowerSpeed);
    }

    //Switch to simple mode, experimental
    if (joy.getRawButtonPressed(5)) {
      simple=true ? (simple = false) : (simple = true);
      System.out.println("Simple: " + simple);
    }

    //Moving coral aimer HOTAS
    if (joy.getRawButton(2)) {
      towerSub.setDifferenceRotation(-0.3);
      // towerSub.runCoralRotateMotors(-0.5);
    } else if (joy.getRawButton(3)) {
      towerSub.setDifferenceRotation(0.3);
      // towerSub.runCoralRotateMotors(0.5);
    } else {
      // towerSub.runCoralRotateMotors(0);
    }

    //Spitting/Sucking algae HOTAS
    if (joy.getRawButton(6)) {
      towerSub.runAlgaeMotors(1);
    } else if (joy.getRawButton(7)) {
      towerSub.runAlgaeMotors(-1);
    } else {
      towerSub.runAlgaeMotors(0);
    }

    //Spitting/Sucking coral HOTAS
    if (joy.getRawButton(11)) {
      towerSub.runCoralMotors(1);
    } else if (joy.getRawButton(10)) {
      towerSub.runCoralMotors(-1);
    }

    //Reset elevator encoders
    if (joy.getRawButton(8) && joy.getRawButton(9)) {
      towerSub.zeroElevatorEncoders();
    }

    if (joy.getRawButton(1)) {
      towerSub.setRotation(27.4);
    }

    //#######      #######
    //####### XBOX #######
    //#######      #######

    //Moving elevator XBOX
    if (!simple) {
      towerSub.setDifferenceHeight(MathUtil.applyDeadband(xbox.getLeftY(), 0.03) * Constants.MaxTowerSpeed);
    } else {
      towerSub.runElevatorMotors(MathUtil.applyDeadband(xbox.getLeftY(), 0.03) * Constants.MaxTowerSpeed);
    }

    //Moving coral aimer XBOX
    // towerSub.setDifferenceRotation(xbox.getRightY() * 0.05);

    //Spitting sucking coral XBOX
    // if (xbox.getLeftTriggerAxis() > 0) {
    //   towerSub.runAlgaeMotors(1);
    // } else if (xbox.getLeftBumperButton()) {
    //   towerSub.runAlgaeMotors(-1);
    // } else {
    //   towerSub.runAlgaeMotors(0);
    // }

    //Spitting/Sucking coral XBOX
    // if (xbox.getRightTriggerAxis() > 0) {
    //   towerSub.runCoralMotors(1);
    // } else if (xbox.getRightBumperButton()) {
    //   towerSub.runCoralMotors(-1);
    // }
    
    //Placing stuff 0 = L4, 270 = L3, 90 = L2, 180 = L1
    if (xbox.getYButtonPressed()) {
      towerSub.setHeight(-20);
      towerSub.setRotation(60.45);
    } else if (xbox.getBButtonPressed()) {
      towerSub.setRotation(68.2);
      towerSub.setHeight(-7.466);
    } else if (xbox.getAButtonPressed()) {
      towerSub.setHeight(-2.5238);
      towerSub.setRotation(68.2);
    } else if (xbox.getXButtonPressed()) {
      towerSub.setHeight(-0.0714);
      towerSub.setRotation(27.4);
    }
    
    //Removing algae Y = top, A = bottom
    // if (xbox.getYButtonPressed()) {
    //   towerSub.setHeight(-15.11);
    // } else if (xbox.getAButtonPressed()) {
    //   towerSub.setHeight(-8.19);
    // }

    // //Recieving coral, B button
    // if (xbox.getBButtonPressed()) {
    //   towerSub.setHeight(-0.0714);
    //   towerSub.setRotation(27.4);
    // }

    // //Placing algae, X button
    // if (xbox.getXButtonPressed()) {
    //   towerSub.setHeight(0);
    // }

    //Code for homing command is in robot container
    
    
  }

  @Override
  public void end(boolean interrupted) {
    towerSub.runElevatorMotors(0);;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
