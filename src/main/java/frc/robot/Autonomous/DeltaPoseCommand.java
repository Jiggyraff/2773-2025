// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Information.LaserSubsystem;
import frc.robot.Information.OdometrySubsystem;
import frc.robot.SwerveSubsystems.DriveSubsystem;

public class DeltaPoseCommand extends Command {

  double goalX;
  double goalY;
  double goalRotation;
  double goalDistance;
  double initY = 0.0;
  double initX = 0.0;
  double initDistance;
  double distanceLeft;
  final double speedTolerance = 0.1;     //Tolerance of fianl position coordinate in meters
  final double rotateTolerance = 5* Math.PI/180;     //Tolerance of fianl position coordinate in meters
  double speed;
  PIDController speedPID = new PIDController(0.63, 0, 0);
  PIDController rotationPID = new PIDController(0.3, 0, 0);
  DriveSubsystem driveSubsystem;
  OdometrySubsystem odomSub;
  

  /** Creates a new MovePolarCommand. */
  public DeltaPoseCommand(double goalX, double goalY, double goalRotation, DriveSubsystem driveSubsystem, OdometrySubsystem odomSub) {
    addRequirements(driveSubsystem);
    this.goalX = goalX;
    this.goalY = goalY;
    this.goalRotation = goalRotation;
    this.driveSubsystem = driveSubsystem;
    this.odomSub = odomSub;
    speedPID = driveSubsystem.getPID();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    goalDistance = Math.sqrt(Math.pow(goalX, 2) + Math.pow(goalY, 2));
    speedPID.setSetpoint(0);
    speedPID.setTolerance(speedTolerance);
    rotationPID.setSetpoint(0);
    rotationPID.setTolerance(rotateTolerance);
    initX = odomSub.getX();
    initY = odomSub.getY();
    goalX += initX;
    goalY += initY;
  }
  
  // Called every time the schetduler runs while the command is scheduled.
  int n = 0;
  double speedSum = 0.0;
  double sumX = 0.0;
  double sumY = 0.0;
  @Override
  public void execute() {
    // double rateOfChange = currentDistance - Math.sqrt(Math.pow(odomSub.getX(), 2)+Math.pow(odomSub.getY(), 2));
    // currentDistance = Math.sqrt(Math.pow(odomSub.getX(), 2)+Math.pow(odomSub.getY(), 2));
    var dx = (goalX - odomSub.getX()); var dy = (goalY - odomSub.getY());
    distanceLeft = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

    double radians = Math.atan2(dy,-dx) - odomSub.getGyroAngle();
    var dr = -(goalRotation - odomSub.getGyroAngle());
    if (dr > Math.PI) {
      dr -= 2*Math.PI;
    } else if (dr < -Math.PI) {
      dr += 2*Math.PI;
    }
    double rspeed = MathUtil.clamp(rotationPID.calculate(dr), -Constants.MaxRotationSpeed, Constants.MaxRotationSpeed);

    speed = MathUtil.clamp(speedPID.calculate(distanceLeft), -Constants.MaxDriveSpeed, Constants.MaxDriveSpeed);
    if (speedPID.atSetpoint()) {
      speed = 0;
    }
    System.out.println("Radians: " + radians + " Rotation: " + odomSub.getGyroAngle() + " Distance Left: " + distanceLeft + "Dx: " + odomSub.getX() + "Dy: " + odomSub.getY());
    driveSubsystem.directionalDrive(speed, radians, rspeed);
    
    speedSum += speed;
    sumX += dx;
    sumY += dy;
    n++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
    System.out.println("##### Move Polar at Destination, average step:" + goalY/n + "; average speed:" + speedSum/n + "; average X:" + sumX/n + "; average Y:" + sumY/n + "DT:" + distanceLeft);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println("Speed: " + speedPID.atSetpoint());
    return speedPID.atSetpoint() && rotationPID.atSetpoint();
  }
}
