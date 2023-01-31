// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.drivers.NavX;

public class DrivetrainSubsystem extends SubsystemBase {

    // public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxSpeed = 0.3;
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  
    // units of rotation2d probably meters (Ocean) 28.5 in, 18.5 in axis to axis (CAD)
    private final Translation2d m_frontLeftLocation = new Translation2d(0.36195, 0.23495);
    private final Translation2d m_frontRightLocation = new Translation2d(0.36195, -0.23495);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.36195, 0.23495);
    private final Translation2d m_backRightLocation = new Translation2d(-0.36195, -0.23495);
    
    public final SwerveModule m_frontLeft = new SwerveModule(Constants.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, 
                                                              Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, 
                                                              Constants.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER, 
                                                              Constants.FRONT_LEFT_ANGLE_OFFSET_COMPETITION,
                                                              Constants.DRIVETRAIN_FRONT_LEFT_ANALOGID);
    public final SwerveModule m_frontRight = new SwerveModule(Constants.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, 
                                                              Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR, 
                                                              Constants.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER, 
                                                              Constants.FRONT_RIGHT_ANGLE_OFFSET_COMPETITION,
                                                              Constants.DRIVETRAIN_FRONT_RIGHT_ANALOGID);
    public final SwerveModule m_backLeft = new SwerveModule(Constants.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, 
                                                              Constants.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR, 
                                                              Constants.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER, 
                                                              Constants.BACK_LEFT_ANGLE_OFFSET_COMPETITION,
                                                              Constants.DRIVETRAIN_BACK_LEFT_ANALOGID);
    public final SwerveModule m_backRight = new SwerveModule(Constants.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, 
                                                              Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR, 
                                                              Constants.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER, 
                                                              Constants.BACK_RIGHT_ANGLE_OFFSET_COMPETITION,
                                                              Constants.DRIVETRAIN_BACK_RIGHT_ANALOGID);
  
    // private final AnalogGyro m_gyro = new AnalogGyro(0);
  
    private final NavX m_gyro = new NavX(SPI.Port.kMXP);
  
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation,
      m_frontRightLocation, 
      m_backLeftLocation, 
      m_backRightLocation);
  
    public final SwerveDriveOdometry m_odometry =
        new SwerveDriveOdometry(
            m_kinematics,
            m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_backLeft.getPosition(),
              m_backRight.getPosition()
            });

  /** Creates a new DrivetrianSubsystem. */
  public DrivetrainSubsystem() {
    System.out.println(m_gyro.getRotation2d());
    m_gyro.calibrate();
    System.out.println(m_gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
    SmartDashboard.putNumber("FL rotation", m_frontLeft.getTurningEncoderRadians());
    SmartDashboard.putNumber("FR rotation", m_frontRight.getTurningEncoderRadians());
    SmartDashboard.putNumber("BL rotation", m_backLeft.getTurningEncoderRadians());
    SmartDashboard.putNumber("BR rotation", m_backRight.getTurningEncoderRadians());

  }

  public void recalibrateGyro() {
    System.out.println(m_gyro.getRotation2d());
    m_gyro.calibrate();
    System.out.println(m_gyro.getRotation2d());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  /**
   * Zeroes the robot's position on the field using SwerveDriveOdometry's resetPosition()
   *
   * <p>Should be used to zero the drivetrain odometry whenever the robot is re-enabled. 
   * Otherwise the robot will "save" the position it was at when it was last disabled, and when
   * re-enabling the robot will think it's at the old position.
  */
  public void zeroDrivetrainOdometry() {
    SwerveModulePosition[] initSwerveModulePos = {new SwerveModulePosition(), 
                                                  new SwerveModulePosition(),
                                                  new SwerveModulePosition(),
                                                  new SwerveModulePosition()};

    this.m_odometry.resetPosition(new Rotation2d(), initSwerveModulePos, new Pose2d());
  }

  
}
