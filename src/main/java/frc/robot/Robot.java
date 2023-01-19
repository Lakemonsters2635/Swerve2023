// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.ResetSwerveGyroCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.SwerveModule;

public class Robot extends TimedRobot {
  public static Joystick rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_CHANNEL);
  public static Joystick leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_CHANNEL);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private static Trigger recalibrateButton = new JoystickButton(rightJoystick, 3);

  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  private final ResetSwerveGyroCommand m_resetSwerveGyroCommand = new ResetSwerveGyroCommand(m_drivetrainSubsystem);

  public Robot() {
    recalibrateButton.onTrue(m_resetSwerveGyroCommand);
  }

  private void configureButtonBindings() {
    
  }

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_drivetrainSubsystem.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    m_drivetrainSubsystem.m_frontLeft.updateSwerveTable(); // 0 analog ID 
    m_drivetrainSubsystem.m_frontRight.updateSwerveTable(); // 3 analog ID
    m_drivetrainSubsystem.m_backLeft.updateSwerveTable(); // 1 analog ID
    m_drivetrainSubsystem.m_backRight.updateSwerveTable(); //2 analog ID
    driveWithJoystick(true);
    // System.out.println("front left table ang: " + m_drivetrainSubsystem.m_frontLeft.t_turningEncoder.getDouble(-1));
    // System.out.println("front right table ang: " + m_drivetrainSubsystem.m_frontRight.t_turningEncoder.getDouble(-1));
    // System.out.println("back left table ang: " + m_drivetrainSubsystem.m_backLeft.t_turningEncoder.getDouble(-1));
    // System.out.println("back right table ang: " + m_drivetrainSubsystem.m_backRight.t_turningEncoder.getDouble(-1));
  }

  public void testPeriodic() {
    m_drivetrainSubsystem.m_frontLeft.updateSwerveTable(); // 0 analog ID 
    m_drivetrainSubsystem.m_frontRight.updateSwerveTable(); // 3 analog ID
    m_drivetrainSubsystem.m_backLeft.updateSwerveTable(); // 1 analog ID
    m_drivetrainSubsystem.m_backRight.updateSwerveTable(); //2 analog ID
    System.out.println("front left table ang: " + m_drivetrainSubsystem.m_frontLeft.t_turningEncoder.getDouble(-1));
    System.out.println("front right table ang: " + m_drivetrainSubsystem.m_frontRight.t_turningEncoder.getDouble(-1));
    System.out.println("back left table ang: " + m_drivetrainSubsystem.m_backLeft.t_turningEncoder.getDouble(-1));
    System.out.println("back right table ang: " + m_drivetrainSubsystem.m_backRight.t_turningEncoder.getDouble(-1));
  }

  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(rightJoystick.getY(), 0.02))
            * DrivetrainSubsystem.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(rightJoystick.getX(), 0.02))
            * DrivetrainSubsystem.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
    //must be positive to read accuate joystick yaw
        m_rotLimiter.calculate(MathUtil.applyDeadband(leftJoystick.getX(), 0.02))
            * DrivetrainSubsystem.kMaxAngularSpeed;

    m_drivetrainSubsystem.drive(xSpeed, ySpeed, rot, fieldRelative);
  }
}
