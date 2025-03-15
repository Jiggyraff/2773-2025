package frc.robot.Information;

import java.lang.System.Logger;
import java.lang.reflect.Field;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveSubsystems.DriveSubsystem;
import frc.robot.SwerveSubsystems.SwerveDriveModule;

public class OdometrySubsystem extends SubsystemBase {

    DriveSubsystem driveSub;
    // Locations for the swerve drive modules relative to the robot center.
    Translation2d m_frontLeftLocation = new Translation2d(0.305, 0.305);
    Translation2d m_frontRightLocation = new Translation2d(0.305, -0.305);
    Translation2d m_backLeftLocation = new Translation2d(-0.305, 0.305);
    Translation2d m_backRightLocation = new Translation2d(-0.305, -0.305);

    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    SwerveDriveOdometry m_odometry;
    SwerveDriveModule[] modules;
    Pose2d pose = new Pose2d();
    Field2d field = new Field2d();

    public OdometrySubsystem(DriveSubsystem driveSub) {
        gyro.reset();
        this.driveSub = driveSub;
        modules = driveSub.modules;
        m_odometry = new SwerveDriveOdometry(
                m_kinematics, gyro.getRotation2d(),
                driveSub.getPositions(),
                new Pose2d(0.0, 0.0, new Rotation2d()));

        Shuffleboard.getTab("Odometry").addDouble("Robot X", () -> {return getX();});
        Shuffleboard.getTab("Odometry").addDouble("Robot Y", () -> {return getY();});

        StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
          .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
        SmartDashboard.putNumber("X", pose.getX());
        SmartDashboard.putNumber("Y", pose.getY());
    }
    
    @Override
    public void periodic() {
        Rotation2d gyroAngle = new Rotation2d(gyro.getAngle() * Math.PI / 180);
        pose = m_odometry.update(gyroAngle,
        driveSub.getPositions());
        field.setRobotPose(pose);
        SmartDashboard.putData("Field", field);
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

    public Pose2d getPose() {
        return pose;
    }

    public void resetPose(Pose2d newPose) {
        pose = newPose;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(driveSub.getStates());
    }

    public double getX() {
        return pose.getX();
    }

    public double getY() {
        return -pose.getY();
    }

    public void setPose(double x, double y, double rotation) {
        pose = new Pose2d(x, y, new Rotation2d(rotation));
        m_odometry.resetPose(pose);
    }

    public void resetGyro() {
        gyro.reset();
    }

    public double[] getSwerveAngles() {
       double[] pos = new double[4];
       for (int i = 0; i < 4; i++) {
        pos[i] = modules[i].canCoderPositionAdjusted();
       }
       return pos;
    }
}
