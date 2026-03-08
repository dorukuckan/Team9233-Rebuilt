// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {

  private final Servo hoodServo;
  private double targetAngle = 0;
  private double currentDistance = 0;
  private boolean enabled = false;

  // Test verileri - Mesafe (metre) -> Açı (derece)
  private static final double[][] ANGLE_MAP = {
    {0.5, 5}, // 0.5 metre -> 5 derece
    {1.0, 10}, // 1.0 metre -> 10 derece
    {1.5, 15}, // 1.5 metre -> 15 derece
    {2.0, 18}, // 2.0 metre -> 18 derece
    {2.5, 22}, // 2.5 metre -> 22 derece
    {3.0, 25}, // 3.0 metre -> 25 derece
    {3.5, 28}, // 3.5 metre -> 28 derece
    {4.0, 30}, // 4.0 metre -> 30 derece
    {4.5, 32}, // 4.5 metre -> 32 derece
    {5.0, 34}, // 5.0 metre -> 34 derece
    {5.5, 36}, // 5.5 metre -> 36 derece
    {6.0, 37}, // 6.0 metre -> 37 derece
    {6.5, 38}, // 6.5 metre -> 38 derece
    {7.0, 39}, // 7.0 metre -> 39 derece
    {7.5, 40}, // 7.5 metre -> 40 derece
    {8.0, 40.5},
    {8.5, 41},
    {9.0, 42},
    {9.5, 42.5},
  };

  public Hood() {
    hoodServo = new Servo(HoodConstants.kHoodServoChannel);
    setAngle(ANGLE_MAP[0][1]);
  }

  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
    Logger.recordOutput("Hood/Enabled", enabled);

    if (!enabled) {
      Logger.recordOutput("Hood/Status", "Disabled - Holding position");
    } else {
      Logger.recordOutput("Hood/Status", "Enabled - Auto adjusting");
    }
  }

  public void setDistance(double distance) {
    currentDistance = distance;

    if (enabled) {
      targetAngle = interpolateAngle(distance);
      setAngle(targetAngle);
      Logger.recordOutput("Hood/Adjusting", true);
    } else {
      Logger.recordOutput("Hood/Adjusting", false);
    }

    Logger.recordOutput("Hood/InputDistance", distance);
    Logger.recordOutput("Hood/TargetAngle", targetAngle);
  }

  private void setAngle(double angle) {
    angle = Math.max(HoodConstants.kMinAngle, Math.min(HoodConstants.kMaxAngle, angle));
    double servoPosition = angle / 180.0;
    hoodServo.set(servoPosition);

    Logger.recordOutput("Hood/Angle", angle);
    Logger.recordOutput("Hood/ServoPosition", servoPosition);
  }

  private double interpolateAngle(double distance) {
    if (distance <= ANGLE_MAP[0][0]) {
      return ANGLE_MAP[0][1];
    }

    if (distance >= ANGLE_MAP[ANGLE_MAP.length - 1][0]) {
      return ANGLE_MAP[ANGLE_MAP.length - 1][1];
    }

    for (int i = 0; i < ANGLE_MAP.length - 1; i++) {
      double dist1 = ANGLE_MAP[i][0];
      double dist2 = ANGLE_MAP[i + 1][0];
      double angle1 = ANGLE_MAP[i][1];
      double angle2 = ANGLE_MAP[i + 1][1];

      if (distance >= dist1 && distance <= dist2) {
        double ratio = (distance - dist1) / (dist2 - dist1);
        return angle1 + ratio * (angle2 - angle1);
      }
    }

    return findClosestAngle(distance);
  }

  private double findClosestAngle(double distance) {
    double closestDist = Double.MAX_VALUE;
    double closestAngle = ANGLE_MAP[0][1];

    for (double[] point : ANGLE_MAP) {
      double distDiff = Math.abs(point[0] - distance);
      if (distDiff < closestDist) {
        closestDist = distDiff;
        closestAngle = point[1];
      }
    }

    return closestAngle;
  }

  public void setManualAngle(double angle) {
    targetAngle = angle;
    setAngle(angle);
    Logger.recordOutput("Hood/ManualAngle", angle);
    Logger.recordOutput("Hood/ManualMode", true);
  }

  public double getCurrentAngle() {
    return hoodServo.get() * 180.0;
  }

  public double getTargetAngle() {
    return targetAngle;
  }

  public double getCurrentDistance() {
    return currentDistance;
  }

  public boolean isEnabled() {
    return enabled;
  }

  public void addCalibrationPoint(double distance, double angle) {
    Logger.recordOutput("Hood/Calibration/Distance", distance);
    Logger.recordOutput("Hood/Calibration/Angle", angle);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Hood/CurrentAngle", getCurrentAngle());
    Logger.recordOutput("Hood/TargetAngle", targetAngle);
    Logger.recordOutput("Hood/Distance", currentDistance);
    Logger.recordOutput("Hood/Enabled", enabled);
    Logger.recordOutput("Hood/IsAdjusting", enabled && currentDistance > 0);
  }
}
