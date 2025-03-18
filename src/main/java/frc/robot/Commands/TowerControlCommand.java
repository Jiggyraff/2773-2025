// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OtherSubsystems.TowerSubsystem;

public class TowerControlCommand extends Command {
  TowerSubsystem towerSub;
  Joystick joy;
  boolean simple;

  /** Creates a new TowerControlCommand. */
  public TowerControlCommand(TowerSubsystem towerSub, Joystick joy) {
    addRequirements(towerSub);
    this.towerSub = towerSub;
    this.joy = joy;
  }

  @Override
  public void initialize() {
    towerSub.setAutomatic(true);
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
    }
    // System.out.println(joy.getRawButton(2));
    if (joy.getRawButton(6)) {
      towerSub.setAlgaeMotors(1);
    } else if (joy.getRawButton(7)) {
      towerSub.setAlgaeMotors(-1);
    } else {
      towerSub.setAlgaeMotors(0);
    }

    if (joy.getRawButton(11)) {
      towerSub.setCoralMotors(1);
    } else if (joy.getRawButton(10)) {
      towerSub.setCoralMotors(-1);
    }

    if (joy.getRawButton(2)) {
      towerSub.setDifferenceRotation(0.05);;
    } else if (joy.getRawButton(3)) {
      towerSub.setDifferenceRotation(-0.05);
    } else {
      towerSub.setDifferenceRotation(0);
    }

    if (joy.getRawButton(8) && joy.getRawButton(9)) {
      towerSub.zeroElevatorEncoders();
    }
    

  }

  @Override
  public void end(boolean interrupted) {
    towerSub.stopElevatorMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
