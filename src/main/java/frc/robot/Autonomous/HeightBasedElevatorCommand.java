// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OtherSubsystems.TowerSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HeightBasedElevatorCommand extends Command {

  TowerSubsystem towerSub;
  double h;

  /** Creates a new MoveElevatorCommand. */
  public HeightBasedElevatorCommand(double h, TowerSubsystem towerSub) {
    // addRequirements(towerSub);
    this.towerSub = towerSub;
    this.h = h;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    towerSub.percentageHeight(h);
    // System.out.println(h);
  }
  @Override
  public boolean isFinished() {
    return towerSub.elevatorAtSetpoint();
  }
}
