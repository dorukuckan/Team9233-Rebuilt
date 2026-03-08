// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  public Robot() {
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    switch (Constants.currentMode) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.start();
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    robotContainer.periodic();
    updatePoseFromLimelight();
  }

  private void updatePoseFromLimelight() {
    try {
      double omegaRps =
          Units.degreesToRotations(robotContainer.drive.getMaxAngularSpeedRadPerSec());
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

      if (llMeasurement != null) {
        Logger.recordOutput("Limelight/TagCount", llMeasurement.tagCount);
        Logger.recordOutput("Limelight/Pose", llMeasurement.pose);
        Logger.recordOutput("Limelight/Timestamp", llMeasurement.timestampSeconds);

        if (llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
          robotContainer.drive.setPose(llMeasurement.pose);
          Logger.recordOutput("Limelight/PoseUpdated", true);
          Logger.recordOutput("Limelight/OmegaRPS", omegaRps);
        } else {
          Logger.recordOutput("Limelight/PoseUpdated", false);
          if (llMeasurement.tagCount == 0) {
            Logger.recordOutput("Limelight/NoTagReason", "No tags detected");
          } else if (Math.abs(omegaRps) >= 2.0) {
            Logger.recordOutput(
                "Limelight/NoTagReason", "Robot moving too fast: " + omegaRps + " RPS");
          }
        }

        Logger.recordOutput("Limelight/Valid", true);
      } else {
        Logger.recordOutput("Limelight/Valid", false);
        Logger.recordOutput("Limelight/TagCount", 0);
        Logger.recordOutput("Limelight/PoseUpdated", false);
        Logger.recordOutput("Limelight/NoTagReason", "No measurement from limelight");
      }
    } catch (Exception e) {
      Logger.recordOutput("Limelight/Valid", false);
      Logger.recordOutput("Limelight/Error", e.getMessage());
      Logger.recordOutput("Limelight/PoseUpdated", false);
    }
  }

  @Override
  public void disabledInit() {
    robotContainer.turret.stopAutoAim();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    robotContainer.turret.stopAutoAim();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    robotContainer.turret.stopAutoAim();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    robotContainer.turret.stopAutoAim();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
