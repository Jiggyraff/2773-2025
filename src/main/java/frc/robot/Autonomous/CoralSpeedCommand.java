// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OtherSubsystems.TowerSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralSpeedCommand extends Command {

  TowerSubsystem towerSub;
  double d;

  /** Creates a new CoralSpeedCommand. */
  public CoralSpeedCommand(TowerSubsystem towerSub, double d) {
    // addRequirements(towerSub);
    this.towerSub = towerSub;
    this.d = d;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    towerSub.runCoralMotors(d);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
