// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int kDriverControllerPort = 0;
  //Measurements of robot
  public static final double DistanceBetweenWheels = 0.616;
  public static final double WheelCircumference =  Math.PI * 0.1016;

  //MotorType of our Neos & PWMs
  public static final MotorType motorType = MotorType.kBrushless;
  //CANIDs for Swerve Modules
        //Drive Motors
        public static final int frontLeftModuleDriveCANID = 17;
        public static final int frontRightModuleDriveCANID = 19;
        public static final int backLeftModuleDriveCANID = 10;
        public static final int backRightModuleDriveCANID = 22;
            //Rotation Motors
        public static final int frontLeftModuleRotateCANID = 16;
        public static final int frontRightModuleRotateCANID = 12;
        public static final int backLeftModuleRotateCANID = 11;
        public static final int backRightModuleRotateCANID = 23;    
            //CTRE CANCoders (Encoders)
        public static final int frontLeftModuleEncoderCANID = 52;
        public static final int frontRightModuleEncoderCANID = 53;
        public static final int backLeftModuleEncoderCANID = 54;
        public static final int backRightModuleEncoderCANID = 55;
    
        public static final double MaxDriveSpeed = 0.6;
        public static final double MaxRotationSpeed = 0.20;
        public static final double MaxTowerSpeed = 0.3;
  //Soft limitations
  public static final double maxWheelSpeed = 1.0; //meters per second

  
  public static final double ControllerDeadzone = 0.1;
  public static final double HOTASDeadzone = 0.1;
  public static final double HOTASRotationDeadzone = 0.3;
}
