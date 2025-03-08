// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Information;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.InputStream;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import edu.wpi.first.wpilibj.SerialPort;
import java.lang.reflect.Type;
import java.nio.channels.DatagramChannel;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveSubsystems.DriveSubsystem;
import frc.robot.SwerveSubsystems.SwerveDriveModule;

public class LaserSubsystem extends SubsystemBase {
  
  DriveSubsystem driveSub;
  OdometrySubsystem odomSub;


  public AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  private static I2C Wire = new I2C(Port.kOnboard, 4);//uses the i2c port on the RoboRIO
  private static final int MAX_BYTES = 32;
  double laserDistance;
  
  //Misc Variables
  // private SerialPort arduino;
  private StringBuilder receivedData; // Use StringBuilder for efficient string building
  SparkMax motor = new SparkMax(14, frc.robot.Constants.motorType);

  
  
  /** Creates a new KinematicsSubsystem. */
  public LaserSubsystem(DriveSubsystem driveSub, OdometrySubsystem odomSub) {
    Shuffleboard.getTab("Odometry").addDouble("Laser Distance (mm)", () -> {return laserDistance;});
    Shuffleboard.getTab("Odometry").addDouble("Laser Motor (Radians)", () -> {return getMotorPosition();});
    // try {
    //         arduino = new SerialPort(9600, SerialPort.Port.kUSB); // Match baud rate!
    //         receivedData = new StringBuilder();
    //     } catch (Exception e) {
    //         System.err.println("Error initializing serial port: " + e.getMessage());
    //     }

    this.driveSub = driveSub;
    this.odomSub = odomSub;
    setAngle(0);
  }
  RelativeEncoder encoder = motor.getEncoder();
  double setAngle = 0;
  PIDController pid = new PIDController(0.1, 0, 0);
  
  @Override
  public void periodic() {
    // if (arduino != null) {
    // try {
    //   // Check how many bytes are available to read
    //   int bytesAvailable = arduino.getBytesReceived();
    //   // System.out.println(bytesAvailable
    //   if (bytesAvailable > 0) {
    //     byte[] buffer = new byte[bytesAvailable]; // Create a buffer to read into
    //     buffer = arduino.read(bytesAvailable); // Read the bytes
    //     int bytesRead = buffer.length;
    //     // Convert bytes to String (UTF-8 encoding is usually good)
    //     String receivedString = new String(buffer, 0, bytesRead, "UTF-8");
        
    //     // Process the received data (e.g., parse values)
    //     receivedData.append(receivedString); // Append to the StringBuilder
        
    //     // Check if a complete message is received (e.g., based on a delimiter like '\n')
    //     if (receivedData.toString().contains("\n")) {
    //       String completeMessage = receivedData.toString().trim();
    //         try {
    //           float sensorValue = Float.parseFloat(completeMessage);
    //           laserDistance = sensorValue;
    //           tagSub.setDistance(laserDistance);
    //           // System.out.println("Sensor Value: " + sensorValue);
    //         } catch (NumberFormatException e) {
    //             // System.err.println("Error parsing float: " + completeMessage);
    //         }
    //       }
    //       receivedData.setLength(0); // Clear the StringBuilder for the next message
    //     }
            

    // } catch (Exception e) {
    //     System.err.println("Error reading from serial port: " + e.getMessage());
    // }
    // }
    double angle = setAngle + odomSub.getGyroAngle();
    while (angle > Math.PI) {
      angle -= 2*Math.PI;
    }
    while (angle < -Math.PI) {
      angle += 2*Math.PI;
    }
    
    pid.setSetpoint(angle);
    double speed = MathUtil.clamp(pid.calculate(getMotorPosition()), -0.15, 0.15);
    motor.set(speed);
    // System.out.println("Position: " + getMotorPosition() + ", Set Angle: " + pid.getSetpoint());
  }
  
  public void setAngleDifference(double d) {
    setAngle += d;
  }


  public double getMotorPosition() {
    double d = (encoder.getPosition()/10)*Math.PI*2;
    while (d > Math.PI) {
      d -= 2*Math.PI;
    }
    while (d < -Math.PI) {
      d += 2*Math.PI;
    }
    return d;
  }

          
  public void setAngle(double setAngle2) {
    while (setAngle2 > Math.PI) {
      setAngle2 -= 2*Math.PI;
    }
    while (setAngle2 < -Math.PI) {
      setAngle2 += 2*Math.PI;
    }
    setAngle = setAngle2;
  }

  public double getRelativeMotorPosition() {
    return getMotorPosition() - Math.PI/2;
  }

  public Double getData() {
    byte[] data = new byte[MAX_BYTES];//create a byte array to hold the incoming data
    Wire.read(8, MAX_BYTES, data);//use address 4 on i2c and store it in data
    String output = new String(data);//create a string from the byte array
    int pt = output.indexOf((char)255);
    String realOut = (String) output.subSequence(0, pt < 0 ? 0 : pt);
    
    try {
      if (realOut == "none") {
        return 0d;
      } else {
        return Double.parseDouble(realOut);
      }
    } catch (NumberFormatException e) {
      return 0d;
    }

  }

  public void setEncoderZero() {
    encoder.setPosition(0);
  }

  public double getGyroAngle() {
    return odomSub.getGyroAngle();
  }
}
