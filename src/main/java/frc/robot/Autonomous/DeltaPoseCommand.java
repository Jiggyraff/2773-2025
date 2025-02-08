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

public class DeltaPoseCommand extends Command {

  double goalX;
  double goalY;
  double goalRotation;
  double goalDistance;
  double initY = 0.0;
  double initX = 0.0;
  double initDistance;
  double distanceTravelled;
  final double speedTolerance = 0.1;     //Tolerance of fianl position coordinate in meters
  final double rotateTolerance = Math.PI/180;     //Tolerance of fianl position coordinate in meters
  double speed;
  PIDController speedPID = new PIDController(0.63, 0, 0);
  PIDController rotationPID = new PIDController(0.63, 0, 0);
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
    speedPID.setSetpoint(goalDistance);
    speedPID.setTolerance(speedTolerance);
    rotationPID.setSetpoint(goalRotation);
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
    var dx = (odomSub.getX() - initX); var dy = (odomSub.getY() - initY);
    distanceTravelled = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

    double radians = Math.tanh((goalY-odomSub.getY())/(goalX-odomSub.getX()));
    double rspeed = MathUtil.clamp(rotationPID.calculate(odomSub.getGyroAngle()), -0.15, 0.15);

    speed = MathUtil.clamp(speedPID.calculate(distanceTravelled), -0.15, 0.15);
    System.out.println("Radians: " + radians + " Speed: " + speed + " Rotation: " + odomSub.getGyroAngle() + " RSpeed: " + rspeed + " Distance Left: " + (goalDistance - distanceTravelled) + "Speed Error:" + speedPID.getError());
    driveSubsystem.directionalDrive(speed, radians+Math.PI, rspeed);
    
    speedSum += speed;
    sumX += dx;
    sumY += dy;
    n++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
    System.out.println("##### Move Polar at Destination, average step:" + goalY/n + "; average speed:" + speedSum/n + "; average X:" + sumX/n + "; average Y:" + sumY/n + "DT:" + distanceTravelled);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return speedPID.atSetpoint() && rotationPID.atSetpoint();
  }
}
