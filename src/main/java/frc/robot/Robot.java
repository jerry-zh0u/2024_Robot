// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  // PhotonCamera camera = new PhotonCamera("photonvision");

  @Override
  public void robotInit() {
    // time.reset();
    // time.start();
    // System.err.println("-------");
    m_robotContainer = new RobotContainer();
    UsbCamera camera_1 = CameraServer.startAutomaticCapture(0);
    UsbCamera camera_2 = CameraServer.startAutomaticCapture(1);
    // camera_1.setResolution(640, 360);
    // camera_2.setResolution(640, 360);
    camera_1.setFPS(30);
    CameraServer.startAutomaticCapture(camera_1);
    CameraServer.startAutomaticCapture(camera_2);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit(){
    // time.reset(); time.start();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    // SmartDashboard.putNumber("time", time.get());
    // if(time.get() > 3){
    //   m_autonomousCommand.cancel();
    // }
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    // Vision-alignment mode
    // Query the latest result from PhotonVision
    // var result = camera.getLatestResult();
    // double range = 0;
    // if (result.hasTargets()) {
    //             // First calculate range
    //   range =
    //     PhotonUtils.calculateDistanceToTargetMeters(
    //       Constants.LimeLightConstants.CAMERA_HEIGHT_METERS,
    //       Constants.LimeLightConstants.TARGET_HEIGHT_METERS,
    //       Constants.LimeLightConstants.CAMERA_PITCH_RADIANS,
    //       Units.degreesToRadians(result.getBestTarget().getPitch()));

    //             // Use this range as the measurement we give to the PID controller.
    //             // -1.0 required to ensure positive PID controller effort _increases_ range
    //   }
    // SmartDashboard.putNumber("result angle", range);
  }

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
