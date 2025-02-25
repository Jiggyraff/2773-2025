// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Information.OdometrySubsystem;
import frc.robot.SwerveSubsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PerfectPoseDifferenceCommand extends Command {

  double differenceX;
  double differenceY;           //IMPORTANT READ!!! This is an experimental command that can rotate at MOST 90 degrees
  double differenceR;
  double initX;
  double initY;
  double initR;
  double goalX;
  double goalY;
  double goalR;
  double[][] swervePositions = new double[4][2];         //[x, y]
  double[][] goalSwervePositions = new double[4][2];     //[x, y]

  PIDController flDrivePID = new PIDController(0.63, 0, 0);
  PIDController blDrivePID = new PIDController(0.63, 0, 0);
  PIDController brDrivePID = new PIDController(0.63, 0, 0);
  PIDController frDrivePID = new PIDController(0.63, 0, 0);
  PIDController[] drivePID = {flDrivePID, blDrivePID, brDrivePID, frDrivePID};
  PIDController flRotatePID = new PIDController(0.63, 0, 0);
  PIDController blRotatePID = new PIDController(0.63, 0, 0);
  PIDController brRotatePID = new PIDController(0.63, 0, 0);
  PIDController frRotatePID = new PIDController(0.63, 0, 0);
  PIDController[] rotatePID = {flRotatePID, blRotatePID, brRotatePID, frRotatePID};

  DriveSubsystem driveSub;
  OdometrySubsystem odomSub;


  /** Creates a new PerfectPoseDifferenceCommand. */
  public PerfectPoseDifferenceCommand(double dx, double dy, double dr, DriveSubsystem driveSub, OdometrySubsystem odomSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.differenceX = dx;
    this.differenceY = dy;
    this.differenceR = dr;
    this.driveSub = driveSub;
    this.odomSub = odomSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initX = odomSub.getX();
    initY = odomSub.getY();
    initR = odomSub.getGyroAngle();
    goalX = initX + differenceX;
    goalY = initY + differenceY;
    goalR = initR + differenceR;

    for (int i = 0; i < 4; i++) {
      goalSwervePositions[i][0] = Math.cos(Math.PI/4 + Math.PI*(i+1) + goalR) * Constants.DistanceBetweenWheels/2 + goalX;
      goalSwervePositions[i][1] = Math.sin(Math.PI/4 + Math.PI*(i+1) + goalR) * Constants.DistanceBetweenWheels/2 + goalY;
    }

    for (int i = 0; i < 4; i++) {
      swervePositions[i][0] = Math.cos(Math.PI/4 + Math.PI*(i+1) + odomSub.getGyroAngle()) * Constants.DistanceBetweenWheels/2 + odomSub.getX();
      swervePositions[i][1] = Math.sin(Math.PI/4 + Math.PI*(i+1) + odomSub.getGyroAngle()) * Constants.DistanceBetweenWheels/2 + odomSub.getY();
    }
    
    for (int i = 0; i < 4; i++) {
      drivePID[i].setSetpoint(0);
      rotatePID[i].setSetpoint(Math.atan2(goalSwervePositions[i][0] - swervePositions[i][0], goalSwervePositions[i][1] - swervePositions[i][1]));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for (int i = 0; i < 4; i++) {
      swervePositions[i][0] = Math.cos(Math.PI/4 + Math.PI*(i+1) + odomSub.getGyroAngle()) * Constants.DistanceBetweenWheels/2 + odomSub.getX();
      swervePositions[i][1] = Math.sin(Math.PI/4 + Math.PI*(i+1) + odomSub.getGyroAngle()) * Constants.DistanceBetweenWheels/2 + odomSub.getY();
    }
    double[][] swerveStates = new double[4][2];   //[angle, speed]
    for (int i = 0; i < 4; i++) {
      swerveStates[i][0] = Math.atan2(goalSwervePositions[i][0] - swervePositions[i][0], goalSwervePositions[i][1] - swervePositions[i][1]);
      swerveStates[i][1] = drivePID[i].calculate(Math.sqrt(goalSwervePositions[i][0] - swervePositions[i][0] + goalSwervePositions[i][1] - swervePositions[i][1]));
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
