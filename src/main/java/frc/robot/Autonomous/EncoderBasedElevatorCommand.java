// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.OtherSubsystems.TowerSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EncoderBasedElevatorCommand extends InstantCommand {
  TowerSubsystem towerSub;
  double h;

  public EncoderBasedElevatorCommand(double h, TowerSubsystem towerSub) {
    addRequirements(towerSub);
    this.towerSub = towerSub;
    this.h = h;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    towerSub.setHeight(h);
  }
  public boolean isFinished() {
    return towerSub.elevatorAtSetpoint();
  }
}
