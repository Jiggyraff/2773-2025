// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
    if (!simple) {
      towerSub.setDifferenceHeight(MathUtil.applyDeadband(joy.getY(), 0.03) * Constants.MaxTowerSpeed);
    } else {
      towerSub.runElevatorMotors(MathUtil.applyDeadband(joy.getY(), 0.03) * Constants.MaxTowerSpeed);
    }

    if (joy.getRawButtonPressed(5)) {
      simple=true ? (simple = false) : (simple = true);
      System.out.println("Simple: " + simple);
    }
    // System.out.println(joy.getRawButton(2));
    if (joy.getRawButton(6)) {
      towerSub.runAlgaeMotors(1);
    } else if (joy.getRawButton(7)) {
      towerSub.runAlgaeMotors(-1);
    } else {
      towerSub.runAlgaeMotors(0);
    }

    if (joy.getRawButton(11)) {
      towerSub.runCoralMotors(1);
    } else if (joy.getRawButton(10)) {
      towerSub.runCoralMotors(-1);
    }

    if (joy.getRawButtonPressed(2)) {
      towerSub.setDifferenceRotation(-0.05);
    } else if (joy.getRawButtonPressed(3)) {
      towerSub.setDifferenceRotation(0.05);
    }

    if (joy.getRawButton(8) && joy.getRawButton(9)) {
      towerSub.zeroElevatorEncoders();
    }
    
    if (xbox.getPOV() == 0) {
      towerSub.setHeight(-15.11);
    } else if (xbox.getPOV() == 180) {
      towerSub.setHeight(-8.19);
    }

    if (xbox.getYButtonPressed()) {
      towerSub.setHeight(-18.33);
      towerSub.setRotation(16.59);
    }

    if (xbox.getRightStickButtonPressed()) {
      towerSub.setHeight(-10.26);
      towerSub.setRotation(29.85);
    }
    

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
