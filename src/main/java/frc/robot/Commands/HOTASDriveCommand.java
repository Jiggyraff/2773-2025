// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveSubsystems.*;
import frc.robot.Information.*;
import frc.robot.Constants;

public class HOTASDriveCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private final Joystick hotas;
  private final TagSubsystem tagSub;
  private final OdometrySubsystem odomSub;
  private PIDController pid;
  private PIDController rotPID;

  //For setpoint
  private boolean Rsetpoint = false;
  private double rx = 0;
  private double ry = 0.5;

  /** Creates a new DriveCommand. */
  public HOTASDriveCommand(DriveSubsystem driveSub, Joystick hotas, TagSubsystem tagSub, OdometrySubsystem odomSub) {
    this.driveSubsystem = driveSub;
    this.hotas = hotas;
    this.tagSub = tagSub;
    this.odomSub = odomSub;
    this.pid = driveSub.getPID();
    addRequirements(driveSub);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    odomSub.getGyroAngle();
    odomSub.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  double oldT = 0.0;
  double oldG = 0.0;
  @Override
  public void execute() {
    buttonMicroCommands();
    double XAxis = hotas.getX(), YAxis = hotas.getY(), ZAxis = hotas.getZ();
    double rawAngle = Math.atan2(YAxis, XAxis);
    double gyroAngle = odomSub.getGyroAngle();
    double sensitivity = MathUtil.clamp(1 - hotas.getThrottle(), 0.05, 1);
    double setDistance = MathUtil.clamp(Math.sqrt(XAxis * XAxis + YAxis * YAxis)*2, 0,2);
    double rotSpeed;
    if (!Rsetpoint) {
      rotSpeed = (MathUtil.applyDeadband(hotas.getZ(), Constants.HOTASRotationDeadzone)) * sensitivity * Constants.MaxRotationSpeed;
    } else {
      rotSpeed = rotateAroundPoint(rx, ry);
    }
          
    pid.setSetpoint(setDistance);
    double driveSpeed = pid.calculate((driveSubsystem.averageDistanceEncoder()-oldT)*11.24) * Constants.MaxDriveSpeed * sensitivity;


    if (Math.abs(XAxis) < Constants.HOTASDeadzone && Math.abs(YAxis) < Constants.HOTASDeadzone && Math.abs(ZAxis) < Constants.HOTASRotationDeadzone) {
      driveSubsystem.stop();
    } else {
      if (Math.abs(ZAxis) > 0) {
        odomSub.getGyroAngle();
      }
      driveSubsystem.directionalDrive(driveSpeed, rawAngle - gyroAngle, rotSpeed);
    }
    oldT = driveSubsystem.averageDistanceEncoder();
    oldG = odomSub.getGyroAngle();
  }

  public void buttonMicroCommands() {
    if (buttonPressed(7) && buttonOnPress(12)) {
      odomSub.resetGyro();
      System.out.println("Gyro Reset Manually");
    }
    if (buttonPressed(7) && buttonOnPress(11)) {
      odomSub.setPose(0,0,odomSub.getGyroAngle());
      System.out.println("Position Reset Manually");
    }
    if (buttonPressed(7) && buttonOnPress(9)) {
      tagSub.enable();
    }
    if (buttonPressed(7) && buttonOnPress(10)) {
      tagSub.cautiousMode();
    }
    if (buttonPressed(3)) {
      tagSub.setPositionDifference(0.05);
      // System.out.println("lk");
    }
    if (buttonPressed(4)) {
      tagSub.setPositionDifference(-0.05);
      // System.out.println("iiih");
    }
    if (buttonPressed(1)) {
      Rsetpoint=true ? Rsetpoint = false : (Rsetpoint = true);
    }
  }

  public Boolean buttonPressed(int i) {
    return hotas.getRawButton(i);
  }

  public Boolean buttonOnPress(int i) {
    return hotas.getRawButtonPressed(i);
  }

  public double rotateAroundPoint(double rx, double ry) {
    double dx = rx - odomSub.getX();
    double dy = ry - odomSub.getY();
    double angle = Math.atan2(dy, dx);
    rotPID.setSetpoint(angle);
    return rotPID.calculate(odomSub.getGyroAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
