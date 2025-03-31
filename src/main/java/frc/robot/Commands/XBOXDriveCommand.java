// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveSubsystems.*;
import frc.robot.Information.*;
import frc.robot.OtherSubsystems.TowerSubsystem;
import frc.robot.Constants;

public class XBOXDriveCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private final XboxController xbox;
  private final TagSubsystem tagSub;
  private final OdometrySubsystem odomSub;
  private final TowerSubsystem towerSub;

  private PIDController pid;
  private PIDController rotPID;
  double sensitivity = 0.5;

  //For setpoint
  private boolean Rsetpoint = false;
  private double rx = 0;
  private double ry = 0.5;

  /** Creates a new DriveCommand. */
  public XBOXDriveCommand(DriveSubsystem driveSub, XboxController xbox, TagSubsystem tagSub, OdometrySubsystem odomSub, TowerSubsystem towerSub) {
    this.driveSubsystem = driveSub;
    this.xbox = xbox;
    this.tagSub = tagSub;
    this.odomSub = odomSub;
    this.pid = driveSub.getPID();
    this.towerSub = towerSub;
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
    double gyroAngle = odomSub.getGyroAngle();
    double elevatorBias = 0.5 + MathUtil.clamp(0.5 - (Math.abs(towerSub.getElevatorPosition()) / 40), 0, 0.5);
    
    if (xbox.getRawButton(1)) {
      sensitivity = 0.25;
    } else if (xbox.getRawButton(0)) {
      sensitivity = 2;
    }

    if (xbox.getPOV() == 0) {             //Top
      driveSubsystem.directionalDrive(0.2 * sensitivity * elevatorBias, Math.PI/2 - gyroAngle);
    } else if (xbox.getPOV() == 90) {     //Idk
      driveSubsystem.directionalDrive(0.2 * sensitivity * elevatorBias, Math.PI - gyroAngle);
    } else if (xbox.getPOV() == 180) {    //Bottom
      driveSubsystem.directionalDrive(0.2 * sensitivity * elevatorBias, -Math.PI/2 - gyroAngle);
    } else if (xbox.getPOV() == 270) {    //Idk
      driveSubsystem.directionalDrive(0.2 * sensitivity * elevatorBias, 0 - gyroAngle);
    } else {
      double XAxis = xbox.getLeftX(), YAxis = xbox.getLeftY(), RAxis = xbox.getRightX();
      double rawAngle = Math.atan2(YAxis, XAxis);
      double setDistance = MathUtil.clamp(Math.sqrt(XAxis * XAxis + YAxis * YAxis)*2, 0,2);
      double rotSpeed;
      if (!Rsetpoint) {
        rotSpeed = (MathUtil.applyDeadband(RAxis, Constants.ControllerDeadzone)) * sensitivity * Constants.MaxRotationSpeed;
      } else {
        rotSpeed = rotateAroundPoint(rx, ry);
      }
      // System.out.println("Axis: " + RAxis + " Speed: " + rotSpeed);

      pid.setSetpoint(setDistance);
      double driveSpeed = pid.calculate((driveSubsystem.averageDistanceEncoder()-oldT)*11.24) * Constants.MaxDriveSpeed * sensitivity;
      driveSpeed *= elevatorBias;

      if (Math.abs(XAxis) < Constants.ControllerDeadzone && Math.abs(YAxis) < Constants.ControllerDeadzone && Math.abs(RAxis) < Constants.ControllerDeadzone) {
        driveSubsystem.stop();
      } else {
        if (Math.abs(RAxis) > 0) {
          odomSub.getGyroAngle();
        }
        driveSubsystem.directionalDrive(driveSpeed, rawAngle - gyroAngle, rotSpeed);
      }
      oldT = driveSubsystem.averageDistanceEncoder();
      oldG = odomSub.getGyroAngle();
    }
  }

  public void buttonMicroCommands() {
    if (buttonPressed(7) && buttonOnPress(8)) {
      odomSub.resetGyro();
      System.out.println("Gyro Reset Manually");
    }
    // if (buttonPressed(1)) {
    //   Rsetpoint = (Rsetpoint==true) ? false : true;
    // }
    
  }

  public Boolean buttonPressed(int i) {
    return xbox.getRawButton(i);
  }

  public Boolean buttonOnPress(int i) {
    return xbox.getRawButtonPressed(i);
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
