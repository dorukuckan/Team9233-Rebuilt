package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final SparkMax intakeMotor =
      new SparkMax(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);

  private boolean isRunning = false;

  public Intake() {

    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(30);

    intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void forwardIntake() {
    intakeMotor.set(IntakeConstants.kForwardIntakeSpeed);
    isRunning = true;
  }

  public void reverseIntake() {
    intakeMotor.set(IntakeConstants.kReverseIntakeSpeed);
    isRunning = true;
  }

  public void stopIntake() {
    intakeMotor.set(0.0);
    isRunning = false;
  }

  public void toggleIntake() {
    if (isRunning) {
      stopIntake();
    } else {
      forwardIntake();
    }
  }

  public boolean isRunning() {
    return isRunning;
  }

  @Override
  public void periodic() {

    Logger.recordOutput("Intake/Running", isRunning);
    Logger.recordOutput("Intake/Speed", intakeMotor.get());
  }
}
