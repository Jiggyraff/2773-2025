// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Information;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveSubsystems.DriveSubsystem;
import frc.robot.SwerveSubsystems.SwerveDriveModule;

public class NavigationSubsystem extends SubsystemBase {

  DriveSubsystem driveSub;

  double angle;
  double[] previousDistanceValues = {0.0,0.0,0.0,0.0};
  // [0] = x, [1] = y
  double[][] deltaCoordinates = new double[4][2];
  double robotX = 0.0;
  double robotY = 0.0;
  double deltaRobotX = 0;
  double deltaRobotY = 0;
  double miscDeltaDistance = 0;
  public AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  //Misc Variables
  double sensitivity;


  /** Creates a new KinematicsSubsystem. */
  public NavigationSubsystem(DriveSubsystem driveSub) {
    this.driveSub = driveSub;
    // Shuffleboard.getTab("Navigation").addDoubleArray("Rotation", () -> {return new double[] { getGyroAngle() };});
    // Shuffleboard.getTab("Navigation").addDoubleArray("Sensitivity", () -> {return new double[] { sensitivity };});
    // Shuffleboard.getTab("Encoders").addDoubleArray("flMotor", () -> {return new double[] { driveSub.modules[0].CANCoderPositionAdjusted(), driveSub.modules[0].distanceEncoderPosition() };});
    // Shuffleboard.getTab("Encoders").addDoubleArray("frMotor", () -> {return new double[] { driveSub.modules[1].CANCoderPositionAdjusted(), driveSub.modules[1].distanceEncoderPosition() };});
    // Shuffleboard.getTab("Encoders").addDoubleArray("blMotor", () -> {return new double[] { driveSub.modules[2].CANCoderPositionAdjusted(), driveSub.modules[2].distanceEncoderPosition() };});
    // Shuffleboard.getTab("Encoders").addDoubleArray("brMotor", () -> {return new double[] { driveSub.modules[3].CANCoderPositionAdjusted(), driveSub.modules[3].distanceEncoderPosition() };});
    // Shuffleboard.getTab("Kinematics").addDoubleArray("Robot", () -> {return new double[] {robotX, robotY};});
    // Shuffleboard.getTab("Kinematics").addDoubleArray("deltaRobot", () -> {return new double[] {deltaRobotX, deltaRobotY};});
    // Shuffleboard.getTab("Kinematics").addDoubleArray("flMotor", () -> {return new double[] {deltaCoordinates[0][0], deltaCoordinates[0][1]};});
    // Shuffleboard.getTab("Kinematics").addDoubleArray("frMotor", () -> {return new double[] {deltaCoordinates[1][0], deltaCoordinates[1][1]};});
    // Shuffleboard.getTab("Kinematics").addDoubleArray("blMotor", () -> {return new double[] {deltaCoordinates[2][0], deltaCoordinates[2][1]};});
    // Shuffleboard.getTab("Kinematics").addDoubleArray("brMotor", () -> {return new double[] {deltaCoordinates[3][0], deltaCoordinates[3][1]};});
    // Shuffleboard.getTab("Kinematics").addDoubleArray("miscMotor", () -> {return new double[] {miscDeltaDistance};});
    // gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    int i = 0;
    deltaRobotX = 0;
    deltaRobotY = 0;
    for (SwerveDriveModule module : driveSub.modules) {
      double deltaDistance = module.distanceEncoderPosition() - previousDistanceValues[i];
      miscDeltaDistance = module.distanceEncoderPosition() - previousDistanceValues[i];
      previousDistanceValues[i] = module.distanceEncoderPosition();
      deltaCoordinates[i][0] = deltaDistance * Math.cos(-module.canCoderPositionAdjusted() - getGyroAngle() );
      deltaCoordinates[i][1] = deltaDistance * Math.sin(-module.canCoderPositionAdjusted() - getGyroAngle() );
      deltaRobotX += deltaCoordinates[i][0];
      deltaRobotY += deltaCoordinates[i][1];
      i++;
    }
    deltaRobotX = deltaRobotX/4;
    deltaRobotY = deltaRobotY/4;
    robotX += deltaRobotX * 11.028;
    robotY += deltaRobotY * 11.028;
    i++;
  }

  public void setPosition(double x, double y) {
    robotX = x;
    robotY = y;
  }

  public void resetOrientation() {
    gyro.reset();
  }

  public double getGyroAngle() {
    double angle = (gyro.getAngle() - 0) / 180.0 * Math.PI;
    while (angle > Math.PI) {
      angle -= Math.PI * 2;
    }
    while (angle < -Math.PI) {
      angle += Math.PI*2;
    }
    return angle;
  }

  public double averageDistanceTraveled() {
    double averageDistance = 0;
    for (SwerveDriveModule module : driveSub.modules) {
      averageDistance += Math.abs(module.distanceEncoderPosition());
    }
    averageDistance = 26.775 * averageDistance/4;
    return averageDistance;
  }

  public void resetGyro() {
    gyro.reset();
  }

  public void resetPosition() {
    robotX = 0;
    robotY = 0;
  }

  public void displayVariables(double sense) {
    sensitivity = sense;
  }
}
