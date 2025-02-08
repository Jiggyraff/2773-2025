// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Information.LaserSubsystem;
import frc.robot.Information.OdometrySubsystem;
import frc.robot.SwerveSubsystems.DriveSubsystem;

public class PolarMoveCommand extends Command {

  double radians;
  double distance;
  DriveSubsystem driveSubsystem;
  double initDistance;
  final double tolerance = 0.1;     //Tolerance of fianl position coordinate in meters
  double speed;
  PIDController pid = new PIDController(0.63, 0, 0);
  OdometrySubsystem odomSub;
  

  /** Creates a new MovePolarCommand. */
  public PolarMoveCommand(double radians, double distance, DriveSubsystem driveSubsystem, OdometrySubsystem odomSub) {
    addRequirements(driveSubsystem);
    this.radians = radians + odomSub.getGyroAngle();
    this.distance = distance;
    this.driveSubsystem = driveSubsystem;
    this.odomSub = odomSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setSetpoint(distance);
    pid.setTolerance(tolerance);
    initDistance = Math.sqrt(Math.pow(odomSub.getX(), 2)+Math.pow(odomSub.getY(), 2));
    oldY = odomSub.getY();
    oldX = odomSub.getX();
  }
  double oldY = 0.0;
  double oldX = 0.0;
  
  // Called every time the schetduler runs while the command is scheduled.
  int n = 0;
  double speedSum = 0.0;
  double sumX = 0.0;
  double sumY = 0.0;
  double sumC = 0.0;
  @Override
  public void execute() {
    // double rateOfChange = currentDistance - Math.sqrt(Math.pow(odomSub.getX(), 2)+Math.pow(odomSub.getY(), 2));
    // currentDistance = Math.sqrt(Math.pow(odomSub.getX(), 2)+Math.pow(odomSub.getY(), 2));
    var c = Math.sqrt(Math.pow(odomSub.getX(), 2) + Math.pow(odomSub.getY(), 2));
    double distanceDifference = Math.abs(c - initDistance);
    var dx = (odomSub.getX() - oldX); var dy = (odomSub.getY() - oldY);
    var dc = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

    // speed = MathUtil.clamp(pid.calculate(distanceDifference), -0.15, 0.15);
    driveSubsystem.directionalDrive(0.15, radians);
    System.out.println(dx + " , " + dy + " , " + dc + " , " +distanceDifference);
    // driveSubsystem.directionalDrive(speed, radians);
    oldY = odomSub.getY();
    oldX = odomSub.getX();
    speedSum += speed;
    sumX += dx;
    sumY += dy;
    sumC += dc;
    n++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
    System.out.println("##### Move Polar at Destination, average step:" + distance/n + "; average speed:" + speedSum/n + "; average X:" + sumX/n + "; average Y:" + sumY/n + "sumc:" + sumC);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distance - 0.1 < sumC && sumC < distance + 0.1;
  }
}
