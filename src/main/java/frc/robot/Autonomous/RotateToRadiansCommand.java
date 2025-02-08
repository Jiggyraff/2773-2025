// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Information.LaserSubsystem;
import frc.robot.Information.OdometrySubsystem;
import frc.robot.SwerveSubsystems.*;

public class RotateToRadiansCommand extends Command {

  OdometrySubsystem odomSub;
  DriveSubsystem driveSub;
  double radians;

  PIDController rotatePID = new PIDController(0.63, 0, 0);

  public RotateToRadiansCommand(double radians, OdometrySubsystem odomSub, DriveSubsystem driveSubsystem) {
    addRequirements(odomSub, driveSubsystem);
    this.odomSub = odomSub;
    this.driveSub = driveSubsystem;
    this.radians = radians;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotatePID.setSetpoint(radians);
    rotatePID.setTolerance(0.01);
    System.out.println();
    System.out.println("Rotate Robot Going To: " + radians + " , "  + (radians - odomSub.getGyroAngle()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedOfRotation = rotatePID.calculate(odomSub.getGyroAngle());
    speedOfRotation = MathUtil.clamp(speedOfRotation, -0.3, 0.3);
    driveSub.rotate(speedOfRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSub.stop();
    System.out.println("Rotate Robot Ended At:" + odomSub.getGyroAngle());
    System.out.println();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotatePID.atSetpoint();
  }
}
