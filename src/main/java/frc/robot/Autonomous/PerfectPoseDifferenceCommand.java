// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Information.OdometrySubsystem;
import frc.robot.SwerveSubsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PerfectPoseDifferenceCommand extends Command {

  double dx;
  double dy;
  double dr;
  double initX;
  double initY;
  double initR;
  double[][] swervePositions = new double[4][3];

  DriveSubsystem driveSub;
  OdometrySubsystem odomSub;


  /** Creates a new PerfectPoseDifferenceCommand. */
  public PerfectPoseDifferenceCommand(double dx, double dy, double dr, DriveSubsystem driveSub, OdometrySubsystem odomSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dx = dx;
    this.dy = dy;
    this.dr = dr;
    this.driveSub = driveSub;
    this.odomSub = odomSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initX = odomSub.getX();
    initY = odomSub.getY();
    initR = odomSub.getGyroAngle();

    for (int i = 0; i < 4; i++) {
      swervePositions[i][0] = Math.cos(Math.PI/4 + Math.PI*(i+1) + initR) * Constants.DistanceBetweenWheels/2 + initX;
      swervePositions[i][1] = Math.sin(Math.PI/4 + Math.PI*(i+1) + initR) * Constants.DistanceBetweenWheels/2 + initY;
      swervePositions[i][2] = odomSub.getSwerveAngles()[i] + initR;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
