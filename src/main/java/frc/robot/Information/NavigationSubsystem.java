// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Information;

import java.util.function.Supplier;

import com.studica.frc.*;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NavigationSubsystem extends SubsystemBase {
  public AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  public double angle;
  // Locations for the swerve drive modules relative to the robot center.
  Translation2d frontLeftLocation = new Translation2d(-Constants.DistanceBetweenWheels / 2,
      Constants.DistanceBetweenWheels / 2);
  Translation2d frontRightLocation = new Translation2d(Constants.DistanceBetweenWheels / 2,
      Constants.DistanceBetweenWheels / 2);
  Translation2d backLeftLocation = new Translation2d(-Constants.DistanceBetweenWheels / 2,
      -Constants.DistanceBetweenWheels / 2);
  Translation2d backRightLocation = new Translation2d(Constants.DistanceBetweenWheels / 2,
      -Constants.DistanceBetweenWheels / 2);

  // Creating my kinematics object using the module locations
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
  double pitch;
  private Supplier<SwerveModulePosition[]> modulePositions;

  private double x;
  private double y;

  public double savedX;
  public double savedY;

  public double displacementX;
  public double displacementY;

  private SwerveDriveOdometry odometry;
  public Pose2d pose;

//   public OdometrySubsystem odomSub;

  /** Creates a new NavigationSubsystem. */
  public NavigationSubsystem() {
    this.modulePositions = modulePositions;

    Shuffleboard.getTab("Navigation").addDoubleArray("Displacement", () -> {
      return new double[] { displacementX, displacementY };
    });
    Shuffleboard.getTab("Navigation").addDoubleArray("Rotation", () -> {
      return new double[] { angle };
    });

    // odometry = new SwerveDriveOdometry(
    //     kinematics, gyro.getRotation2d(), modulePositions.get(), new Pose2d(0.0, 0.0, new Rotation2d()));
    gyro.reset();
  }

  public double angle() {
    return this.angle;
  }

  public Pose2d pose() {
    return pose;
  }

  public void resetOrientation() {
    gyro.reset();
  }

  @Override
  public void periodic() {
    angle = (gyro.getAngle() - 0) / 180.0 * Math.PI;
    // pose = odometry.update(gyro.getRotation2d(), modulePositions.get());
    // x = pose.getY();
    // y = -pose.getX();
    // displacementX = x - savedX;
    // displacementY = y - savedY;
    // savedX = x;
    // savedY = y;
  }

  public void reset() {
    x = 0;
    y = 0;
  }
}
