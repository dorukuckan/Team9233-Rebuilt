package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {

  private final SparkMax indexerMotor =
      new SparkMax(IndexerConstants.kIndexerMotorID, MotorType.kBrushless);

  private boolean isRunning = false;

  public Indexer() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(30);

    indexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** İndexer ileri çalıştır */
  public void forwardIndexer() {
    indexerMotor.set(IndexerConstants.kForwardIndexerSpeed);
    isRunning = true;
  }

  /** İndexer ters çalıştır */
  public void reverseIndexer() {
    indexerMotor.set(IndexerConstants.kReverseIndexerSpeed);
    isRunning = true;
  }

  /** İndexeri durdur */
  public void stopIndexer() {
    indexerMotor.set(0.0);
    isRunning = false;
  }

  /** Toggle: çalışıyorsa durdur, duruyorsa ileri çalıştır */
  public void toggleIndexer() {
    if (isRunning) {
      stopIndexer();
    } else {
      forwardIndexer();
    }
  }

  /** Indexer durumunu döndür */
  public boolean isRunning() {
    return isRunning;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Indexer/Running", isRunning);
    Logger.recordOutput("Indexer/Speed", indexerMotor.get());
  }
}
