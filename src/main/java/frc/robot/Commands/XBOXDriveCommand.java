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
  private double setX = 0;
  private double setY = 0;
  private boolean teleop = true;

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
  boolean mode = true;
  @Override
  public void execute() {
    if (Math.abs(xbox.getLeftY()) > Constants.ControllerDeadzone) {
      teleop = true;
    }
    buttonMicroCommands();
    double gyroAngle = odomSub.getGyroAngle();
    double elevatorBias = 0.5 + MathUtil.clamp(0.5 - (Math.abs(towerSub.getElevatorPosition()) / 40), 0, 0.5);
    
    if (xbox.getRawButton(1)) {
      sensitivity = 0.25;
    } else if (xbox.getRawButton(2)) {
      sensitivity = 2;
    } else {
      sensitivity = 1;
    }
    double XAxis = 0;
    double YAxis = 0;
    double RAxis = 0;
    double driveAngle = 0;
    double driveSpeed = 0;
    double setDistance = 0;
    if (teleop) {
      if (xbox.getPOV() == 180) {             //Top
        driveAngle = Math.PI/2;
        driveSpeed = 0.05 * sensitivity;
      } else if (xbox.getPOV() == 270) {     //Idk
        driveAngle = Math.PI;
        driveSpeed = 0.05 * sensitivity;
      } else if (xbox.getPOV() == 0) {    //Bottom
        driveAngle = -Math.PI/2;
        driveSpeed = 0.05 * sensitivity;
      } else if (xbox.getPOV() == 90) {    //Idk
        driveAngle = 0;
        driveSpeed = 0.05 * sensitivity;
      } else {
        XAxis = xbox.getLeftX();
        YAxis = xbox.getLeftY();
        RAxis = xbox.getRightX();
        driveAngle = Math.atan2(YAxis, XAxis) - gyroAngle;
        setDistance = MathUtil.clamp(Math.sqrt(XAxis * XAxis + YAxis * YAxis)*2, 0,2);
        pid.setSetpoint(setDistance);
        driveSpeed = pid.calculate((driveSubsystem.averageDistanceEncoder()-oldT)*11.24) * Constants.MaxDriveSpeed * sensitivity;
        driveSpeed *= elevatorBias;
      }
    } else {
      XAxis = setX;
      YAxis = setY;
      driveAngle = Math.atan2(YAxis, XAxis) - gyroAngle;
      setDistance = MathUtil.clamp(Math.sqrt(XAxis * XAxis + YAxis * YAxis)*2, 0,2);
      pid.setSetpoint(setDistance);
      driveSpeed = pid.calculate((driveSubsystem.averageDistanceEncoder()-oldT)*11.24) * Constants.MaxDriveSpeed * sensitivity;
      driveSpeed *= elevatorBias;
    }
    // System.out.println("Speed: " + driveSpeed + " Angle: " + driveAngle);

  double rotSpeed;
  if (!Rsetpoint) {
    rotSpeed = (MathUtil.applyDeadband(RAxis, Constants.ControllerDeadzone)) * sensitivity * Constants.MaxRotationSpeed;
  } else {
    rotSpeed = rotateAroundPoint(rx, ry);
  }
  
  mode = (xbox.getLeftStickButtonPressed()) ? true : false;
  if (mode) {
    rotSpeed *= 1;
  } else {
    rotSpeed *= 0.5;
  }
      // System.out.println("Axis: " + RAxis + " Speed: " + rotSpeed);


      if (Math.abs(XAxis) < Constants.ControllerDeadzone && Math.abs(YAxis) < Constants.ControllerDeadzone && Math.abs(RAxis) < Constants.ControllerDeadzone && xbox.getPOV() != 0 && xbox.getPOV() != 90 && xbox.getPOV() != 270 && xbox.getPOV() != 180) {
        driveSubsystem.stop();
      } else {
        if (Math.abs(RAxis) > 0) {
          odomSub.getGyroAngle();
        }
        driveSubsystem.directionalDrive(driveSpeed, driveAngle, rotSpeed);
      }
      oldT = driveSubsystem.averageDistanceEncoder();
      oldG = odomSub.getGyroAngle();
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

  public void setXYSpeed(double x, double y) {
    teleop = false;
    setX = x;
    setY = y;
  }
}
