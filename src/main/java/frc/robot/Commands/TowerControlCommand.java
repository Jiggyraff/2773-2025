// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OtherSubsystems.TowerSubsystem;

public class TowerControlCommand extends Command {
  TowerSubsystem towerSub;
  Joystick joy;

  /** Creates a new TowerControlCommand. */
  public TowerControlCommand(TowerSubsystem towerSub, Joystick joy) {
    addRequirements(towerSub);
    this.towerSub = towerSub;
    this.joy = joy;
  }

  @Override
  public void initialize() {
    towerSub.setAutomatic(true);
    towerSub.runMotors(0);
  }
  
  @Override
  public void execute() {
    towerSub.setDifferenceHeight(joy.getY() * Constants.MaxTowerSpeed);
    

  }

  @Override
  public void end(boolean interrupted) {
    towerSub.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
