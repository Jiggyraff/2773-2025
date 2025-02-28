// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OtherSubsystems.ClimberSubsystem;
import frc.robot.OtherSubsystems.TowerSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TowerControlCommand extends Command {


  TowerSubsystem towerSub;
  ClimberSubsystem climbSub;
  Joystick joy;

  /** Creates a new TowerControlCommand. */
  public TowerControlCommand(TowerSubsystem towerSub, ClimberSubsystem climbSub, Joystick joy) {
    addRequirements(towerSub);
    this.towerSub = towerSub;
    this.climbSub = climbSub;
    this.joy = joy;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double sensitivity = 2/(joy.getZ()+1);
    towerSub.throttleControl(joy.getZ() * 1);
    
    microCommands();
  }

  public void microCommands() {
    if (joy.getRawButtonPressed(8)) {
      towerSub.zeroEncoders();
      System.out.println("Manually reset tower encoders");
    }
    if (joy.getRawButton(9)) {
      towerSub.setSpeed(0.05);
      System.out.println("Manually moving tower down");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
