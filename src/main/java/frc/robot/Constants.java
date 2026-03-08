// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final class TurretConstants {
    public static final int kTurretMotorID = 15;

    public static final double kMaxTurretAngle = 180;
    public static final double kMinTurretAngle = -180;
  }

  public static final class LauncherConstants {
    public static final int kShooterMotorID = 11; // CAN ID

    // Hedef RPM
    public static final double kShooterRPM = 4000; // Test edilerek ayarlanacak

    // PID değerleri (Phoenix 6 için)
    public static final double kP = 0.1; // Proportional gain
    public static final double kI = 0.0; // Integral gain (genelde 0)
    public static final double kD = 0.0; // Derivative gain (genelde 0)
    public static final double kV = 0.12; // Feed-forward (voltaj / RPS) - önemli!

    // Tolerans
    public static final double kToleranceRPM = 50; // 50 RPM tolerans

    // Farklı mesafeler için RPM değerleri (opsiyonel)
    public static final double kCloseRPM = 3000; // Yakın mesafe
    public static final double kMidRPM = 4000; // Orta mesafe
    public static final double kFarRPM = 5000; // Uzak mesafe
  }

  public static final class HoodConstants {
    public static final int kHoodServoChannel = 0;

    // Servo açı limitleri (fiziksel limitler)
    public static final double kMinAngle = 0; // Minimum açı (derece)
    public static final double kMaxAngle = 60; // Maximum açı (derece) - mekanik limite göre ayarla

    // Varsayılan açı
    public static final double kDefaultAngle = 10; // 1 metre için varsayılan
  }

  public static final class IntakeConstants {
    public static final int kIntakeMotorID = 20;
    public static final double kForwardIntakeSpeed = 0.7;
    public static final double kReverseIntakeSpeed = -0.7;
  }

  public static final class IndexerConstants {
    public static final int kIndexerMotorID = 21;
    public static final double kForwardIndexerSpeed = 0.6;
    public static final double kReverseIndexerSpeed = -0.6;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
