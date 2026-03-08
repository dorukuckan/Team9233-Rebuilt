// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {

  private final SparkMax turretMotor;
  private final RelativeEncoder turretEncoder;
  private final PIDController turretPID;
  private final Hood hood;

  private static final double MIN_ANGLE = -180;
  private static final double MAX_ANGLE = 180;
  
  // Turret'in robot üzerindeki offset'i (robot merkezine göre metre cinsinden)
  private static final Translation2d TURRET_OFFSET = new Translation2d(0.15, 0); // 15 cm önde

  private double lastTargetAngle = 0;
  private Pose2d robotPose = new Pose2d();
  private boolean autoAimEnabled = false;
  private Translation2d currentTarget = null;
  private double currentDistance = 0;

  public Turret(Hood hood) {
    this.hood = hood;
    
    // Turret SparkMax
    turretMotor = new SparkMax(TurretConstants.kTurretMotorID, MotorType.kBrushless);
    turretEncoder = turretMotor.getEncoder();

    SparkMaxConfig turretConfig = new SparkMaxConfig();
    turretConfig.encoder.positionConversionFactor(360.0); // 1 devir = 360 derece
    turretConfig.smartCurrentLimit(30);
    
    // Motor yönünü kontrol et (gerekirse true yap)
    turretConfig.inverted(false);
    
    turretMotor.configure(
        turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // PID katsayıları
    turretPID = new PIDController(0.01, 0.0, 0.001);
    turretPID.enableContinuousInput(MIN_ANGLE, MAX_ANGLE);
    turretPID.setTolerance(2.0); // 2 derece tolerans
    
    zeroEncoder();
  }

  public void setAutoAim(boolean enabled) {
    if (autoAimEnabled != enabled) {
      autoAimEnabled = enabled;
      hood.setEnabled(enabled);
      Logger.recordOutput("Turret/AutoAim", enabled);
      
      if (!enabled) {
        turretMotor.set(0);
      }
    }
  }

  public void startAutoAim() {
    setAutoAim(true);
  }

  public void stopAutoAim() {
    setAutoAim(false);
  }

  public boolean isAutoAimEnabled() {
    return autoAimEnabled;
  }

  public Translation2d getCurrentTarget() {
    return currentTarget;
  }
  
  public double getCurrentDistance() {
    return currentDistance;
  }
  
  /**
   * Turret'in saha üzerindeki gerçek konumunu hesaplar
   */
  public Pose2d getTurretPose() {
    if (robotPose == null) return new Pose2d();
    
    // Robotun rotasyonu ile offset'i döndür
    Rotation2d robotRotation = robotPose.getRotation();
    Translation2d rotatedOffset = TURRET_OFFSET.rotateBy(robotRotation);
    Translation2d turretTranslation = robotPose.getTranslation().plus(rotatedOffset);
    Rotation2d turretRotation = robotRotation.plus(Rotation2d.fromDegrees(turretEncoder.getPosition()));
    
    return new Pose2d(turretTranslation, turretRotation);
  }

  @Override
  public void periodic() {
    if (robotPose == null) return;
    update(robotPose);
  }

  public void update(Pose2d robotPose) {
    this.robotPose = robotPose;
    
    // Turret'in sahadaki pozisyonunu hesapla
    Pose2d turretPose = getTurretPose();
    
    // Hedef belirleme
    if (autoAimEnabled) {
      currentTarget = getTargetFromPose(robotPose);
    } else {
      currentTarget = null;
    }

    double turretAngle = turretEncoder.getPosition();
    
    // Motor kontrolü
    if (currentTarget != null && autoAimEnabled) {
      // Hedefe olan vektörü hesapla
      Translation2d turretTranslation = turretPose.getTranslation();
      double dx = currentTarget.getX() - turretTranslation.getX();
      double dy = currentTarget.getY() - turretTranslation.getY();
      currentDistance = Math.hypot(dx, dy);

      // Hedef açıyı hesapla
      double fieldAngle = Math.toDegrees(Math.atan2(dy, dx));
      double robotHeading = robotPose.getRotation().getDegrees();
      double turretTarget = MathUtil.inputModulus(fieldAngle - robotHeading, MIN_ANGLE, MAX_ANGLE);
      lastTargetAngle = turretTarget;

      // PID çıkışı
      double output = turretPID.calculate(turretAngle, turretTarget);
      output = MathUtil.clamp(output, -0.8, 0.8);
      turretMotor.set(output);

      // Hood'u ayarla
      hood.setDistance(currentDistance);

    } else {
      turretMotor.set(0);
      lastTargetAngle = Double.NaN;
      currentDistance = 0;
    }

    // SADELEŞTİRİLMİŞ LOGLAR - Sadece temel bilgiler
    Logger.recordOutput("Turret/Angle", turretAngle);
    Logger.recordOutput("Turret/TargetAngle", lastTargetAngle);
    Logger.recordOutput("Turret/Distance", currentDistance);
    Logger.recordOutput("Turret/AutoAim", autoAimEnabled);
    Logger.recordOutput("Turret/HasTarget", currentTarget != null);
    Logger.recordOutput("Turret/AtSetpoint", turretPID.atSetpoint());
    
    // Turret pozisyonu - TEK BİR POSE OLARAK
    Logger.recordOutput("Turret/Pose", turretPose);
    
    // Hedef pozisyonu (varsa)
    if (currentTarget != null) {
      Logger.recordOutput("Turret/TargetPose", new Pose2d(currentTarget, new Rotation2d()));
    }
  }

  public void zeroEncoder() {
    turretEncoder.setPosition(0);
    Logger.recordOutput("Turret/Zeroed", true);
  }

  public double getTurretAngle() {
    return turretEncoder.getPosition();
  }

  public double getLastTargetAngle() {
    return lastTargetAngle;
  }

  public boolean isTargeted() {
    return currentTarget != null && autoAimEnabled;
  }
  
  public boolean isAtTarget() {
    return autoAimEnabled && currentTarget != null && turretPID.atSetpoint();
  }
  
  public boolean isReadyToShoot() {
    return isAtTarget() && hood.isEnabled();
  }

  /**
   * Robot pozisyonuna göre hedef seçer
   */
  private Translation2d getTargetFromPose(Pose2d pose) {
    var alliance = DriverStation.getAlliance();
    
    if (alliance.isEmpty()) {
      return null;
    }
    
    double x = pose.getX();
    double y = pose.getY();

    if (alliance.get() == Alliance.Blue) {
      // Mavi ittifak hedefleri
      if (x < 4.4) {
        return new Translation2d(4.62, 4.03); // Speaker
      }
      if (x > 5.3 && x < 11.7 && y > 5) {
        return new Translation2d(2, 6); // Amplifier üst
      }
      if (x > 5.3 && x < 11.7 && y < 3) {
        return new Translation2d(2, 2); // Amplifier alt
      }
    } else if (alliance.get() == Alliance.Red) {
      // Kırmızı ittifak hedefleri
      if (x > 16.54 - 4.4) {
        return new Translation2d(16.54 - 4.62, 4.03); // Speaker
      }
      if (x >= 5.3 && x < 11.7 && y > 5) {
        return new Translation2d(16.54 - 2, 6); // Amplifier üst
      }
      if (x >= 5.3 && x < 11.7 && y < 3) {
        return new Translation2d(16.54 - 2, 2); // Amplifier alt
      }
    }

    return null;
  }
}