// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;
import org.littletonrobotics.junction.Logger;

public class Launcher extends SubsystemBase {

  private final TalonFX shooterMotor;
  private double targetRPM = 0;
  public boolean isEnabled = false;
  private boolean canShoot = false;

  // Phoenix 6'in yerleşik Velocity kontrolü için
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  public Launcher() {
    // TalonFX motor initialization
    shooterMotor = new TalonFX(LauncherConstants.kShooterMotorID);

    // Motor yapılandırması
    TalonFXConfiguration config = new TalonFXConfiguration();

    // PID değerleri (test edilerek ayarlanmalı)
    config.Slot0.kP = LauncherConstants.kP;
    config.Slot0.kI = LauncherConstants.kI;
    config.Slot0.kD = LauncherConstants.kD;
    config.Slot0.kV = LauncherConstants.kV; // Feed-forward (voltaj/Hz)

    // Hız limitleri
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2; // 0.2 saniyede rampa

    // Akım limitleri
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    shooterMotor.getConfigurator().apply(config);
    shooterMotor.setNeutralMode(NeutralModeValue.Coast);

    // Encoder'ı sıfırla (opsiyonel)
    shooterMotor.setPosition(0);
  }

  /** Starts shooter at target RPM */
  public void startShooter() {
    isEnabled = true;
    targetRPM = LauncherConstants.kShooterRPM;
  }

  /** Starts shooter at custom RPM */
  public void startShooter(double rpm) {
    isEnabled = true;
    targetRPM = rpm;
    Logger.recordOutput("Shooter/CustomRPM", rpm);
  }

  /** Stops shooter */
  public void stopShooter() {
    isEnabled = false;
    targetRPM = 0;
    shooterMotor.stopMotor();
  }

  /** Toggle shooter state */
  public void toggleShooter() {
    if (isEnabled) {
      stopShooter();
    } else {
      startShooter();
    }
  }

  public void setCanShoot(boolean canShoot) {
    this.canShoot = canShoot;
  }

  /** Returns if the shooter is running */
  public boolean isEnabled() {
    return isEnabled;
  }

  /** Returns current shooter RPM */
  public double getCurrentRPM() {
    // RPS'den RPM'ye çevir (1 RPS = 60 RPM)
    return shooterMotor.getVelocity().getValueAsDouble() * 60.0;
  }

  /** Returns target RPM */
  public double getTargetRPM() {
    return targetRPM;
  }

  /** Checks if shooter is up to speed */
  public boolean atSpeed() {
    if (!isEnabled || targetRPM == 0) return false;
    double currentRPM = getCurrentRPM();
    double error = Math.abs(currentRPM - targetRPM);
    return error < LauncherConstants.kToleranceRPM;
  }

  /** Checks if shooter is ready to fire */
  public boolean readyToFire() {
    return isEnabled && canShoot && atSpeed();
  }

  /** Gets the motor temperature in Celsius */
  public double getTemperature() {
    return shooterMotor.getDeviceTemp().getValueAsDouble();
  }

  /** Gets the supply current in Amps */
  public double getSupplyCurrent() {
    return shooterMotor.getSupplyCurrent().getValueAsDouble();
  }

  /** Gets the stator current in Amps */
  public double getStatorCurrent() {
    return shooterMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // Motor kontrolü - PID ile
    if (isEnabled && targetRPM > 0) {
      // RPM'i RPS'ye çevir (Phoenix 6 RPS bekler)
      double targetRPS = targetRPM / 60.0;

      // VelocityVoltage kontrolü ile PID + Feed-forward
      shooterMotor.setControl(velocityRequest.withVelocity(targetRPS));

      Logger.recordOutput("Shooter/State", "Running");
      Logger.recordOutput("Shooter/ControlMode", "PID_Velocity");
    } else {
      shooterMotor.stopMotor();
      Logger.recordOutput("Shooter/State", "Stopped");
    }

    // Loglama - getValueAsDouble() kullanıldığına dikkat edin
    double currentRPM = getCurrentRPM();
    Logger.recordOutput("Shooter/CurrentRPM", currentRPM);
    Logger.recordOutput("Shooter/TargetRPM", targetRPM);
    Logger.recordOutput("Shooter/ErrorRPM", targetRPM - currentRPM);
    Logger.recordOutput("Shooter/IsEnabled", isEnabled);
    Logger.recordOutput("Shooter/CanShoot", canShoot);
    Logger.recordOutput("Shooter/AtSpeed", atSpeed());
    Logger.recordOutput("Shooter/ReadyToFire", readyToFire());

    // Motor çekişi (Amper)
    Logger.recordOutput("Shooter/SupplyCurrent", getSupplyCurrent());
    Logger.recordOutput("Shooter/StatorCurrent", getStatorCurrent());
    Logger.recordOutput("Shooter/Temperature", getTemperature());
  }
}
