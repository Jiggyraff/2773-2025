// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OtherSubsystems.TowerSubsystem;

public class SimpleTowerControlCommand extends Command {
private TowerSubsystem towerSub;
private Joystick towerJoy;

  public SimpleTowerControlCommand(TowerSubsystem towerSub, Joystick towerJoy) {
    addRequirements(towerSub);
    this.towerSub = towerSub;
    this.towerJoy = towerJoy;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    towerSub.setAutomatic(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    towerSub.runMotors(towerJoy.getY()*Constants.MaxTowerSpeed*0.6);
    
    if (towerJoy.getRawButton(11)) {
      if (towerJoy.getRawButtonPressed(10)) {
        towerSub.zeroEncoders();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    towerSub.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
