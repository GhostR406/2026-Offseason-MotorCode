// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volt;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedPowerDistribution;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.Ports;


public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
 public static double Time;
 public double autoStartTime;
 public double autoTime;
 public double teleOpStartTime;
 public double teleOpTime;
 private  RobotContainer m_robotContainer;

  @Override
  public void robotInit() {

        // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    //Initalizes Advantage kit logger
    //Testing Branch Commits :eyes:

      DataLogManager.start();

          // Metadata
          Logger.recordMetadata("Code_ProjectName", BuildConstants.PROJECT_NAME);
          Logger.recordMetadata("Code_Version", BuildConstants.VERSION);
          Logger.recordMetadata("Code_BuildDate", BuildConstants.BUILD_DATE);
          Logger.recordMetadata("Git_Version_Changed", BuildConstants.GIT_REVISION);
          Logger.recordMetadata("Git_Branch", BuildConstants.GIT_BRANCH);

          switch (BuildConstants.DIRTY) {
              case 0 -> Logger.recordMetadata("GitDirty", "All changes committed");
              case 1 -> Logger.recordMetadata("GitDirty", "Uncommitted changes");
              default -> Logger.recordMetadata("GitDirty", "Unknown");

          }
                  // Set up data receivers & replay source
          switch (Constants.currentMode) {
              case REAL -> {
                  // Running on a real robot, log to a USB stick ("/U/logs")
                  Logger.addDataReceiver(new WPILOGWriter());
                  Logger.addDataReceiver(new NT4Publisher());
                  LoggedPowerDistribution.getInstance(Ports.REV_pdh.id(), ModuleType.kRev);
              }

              case SIM -> {
                  // Running a physics simulator, log to NT
                  Logger.addDataReceiver(new WPILOGWriter());
                  Logger.addDataReceiver(new NT4Publisher());
              }

              case REPLAY -> {
                  // Replaying a log, set up replay source
                  this.setUseTiming(false); // Run as fast as possible
                  String logPath = LogFileUtil.findReplayLog();
                  Logger
                      .setReplaySource(new WPILOGReader(logPath));
                  Logger
                      .addDataReceiver(
                          new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
              }
          }


          Logger.start();


    //Turn off no controleller warning if not a real robot
    DriverStation.silenceJoystickConnectionWarning(!Robot.isReal());

  }

  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run(); 

    Logger.recordOutput("Robot/Time", Timer.getMatchTime());

    Constants.TurboLikeTheHorseVortexConstants.getMaxRotomIO().runVoltage(Volt.of(12));
  }


  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
  }

  @Override
  public void autonomousPeriodic() {

    //  m_robotContainer.m_aprilTag.updatedPoseFromTagAuto();
  
    
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
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
}

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
