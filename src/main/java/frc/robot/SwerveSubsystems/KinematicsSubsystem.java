// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveSubsystems;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import frc.robot.Constants;

public class KinematicsSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public KinematicsSubsystem(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
  }

  DriveSubsystem driveSubsystem;

  AHRS gyro = new AHRS(hu);
  double gyroAngle;
  
  
  //Whenever swerve modules are passed by a function, they will be in this order:
  //Front Left, Front Right, Back Left, Back Right
  Translation2d[] trans = {
  new Translation2d(-Constants.DistanceBetweenWheels/2, Constants.DistanceBetweenWheels/2),
  new Translation2d(Constants.DistanceBetweenWheels/2, Constants.DistanceBetweenWheels/2),
  new Translation2d(-Constants.DistanceBetweenWheels/2, -Constants.DistanceBetweenWheels/2),
  new Translation2d(Constants.DistanceBetweenWheels/2, -Constants.DistanceBetweenWheels/2),
  };

  SwerveModuleState[] states = {
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState(),
    new SwerveModuleState()};

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(trans);

  SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(gyro.getYaw()), stateToPositions(states));
  Pose2d pose;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    gyroAngle = gyro.getYaw() / 180.0 * Math.PI;
    
    pose = odometry.update(new Rotation2d(gyroAngle), stateToPositions(states));
  }

  public SwerveModulePosition[] stateToPositions(SwerveModuleState[] states) {
    SwerveModulePosition[] pos = {
      new SwerveModulePosition(states[0].speedMetersPerSecond, states[0].angle),
      new SwerveModulePosition(states[1].speedMetersPerSecond, states[1].angle),
      new SwerveModulePosition(states[2].speedMetersPerSecond, states[2].angle),
      new SwerveModulePosition(states[3].speedMetersPerSecond, states[3].angle)
    };
    return pos;
  }
}
