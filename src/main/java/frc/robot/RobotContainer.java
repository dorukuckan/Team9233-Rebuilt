// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems - PUBLIC olarak tanımlandı
  public final Drive drive;
  public final Hood hood;
  public final Launcher launcher;
  public final Turret turret;
  public final Intake intake;
  public final Indexer indexer;

  // Controller
  private final CommandPS4Controller controller = new CommandPS4Controller(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    // Subsystem'leri doğru sırayla oluştur
    hood = new Hood();
    launcher = new Launcher();
    turret = new Turret(hood);
    intake = new Intake();
    indexer = new Indexer();
    
    // Drive subsystem'ünü oluştur
    switch (Constants.currentMode) {
      case REAL:
        turret.zeroEncoder();
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        turret.zeroEncoder();
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        turret.zeroEncoder();
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // Named commands'leri kaydet
    registerNamedCommands();
    
    // Auto chooser'ı oluştur
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // SysId routines ekle
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Button bindings'i yapılandır
    configureButtonBindings();
    
    Logger.recordOutput("RobotContainer/Initialized", true);
  }

  private void registerNamedCommands() {
    // Intake commands
    NamedCommands.registerCommand("IntakeOn", Commands.runOnce(intake::forwardIntake, intake));
    NamedCommands.registerCommand("IntakeOff", Commands.runOnce(intake::stopIntake, intake));
    NamedCommands.registerCommand("IntakeReverse", Commands.runOnce(intake::reverseIntake, intake));
    NamedCommands.registerCommand(
        "ToggleIntake", Commands.runOnce(() -> intake.toggleIntake(), intake));

    // Launcher commands
    NamedCommands.registerCommand(
        "LauncherOn", Commands.runOnce(() -> launcher.startShooter(), launcher));
    NamedCommands.registerCommand(
        "LauncherOff", Commands.runOnce(() -> launcher.stopShooter(), launcher));
    NamedCommands.registerCommand(
        "ToggleLauncher", Commands.runOnce(() -> launcher.toggleShooter(), launcher));
    NamedCommands.registerCommand(
        "LauncherClose", Commands.runOnce(() -> launcher.startShooter(3000), launcher));
    NamedCommands.registerCommand(
        "LauncherMid", Commands.runOnce(() -> launcher.startShooter(4000), launcher));
    NamedCommands.registerCommand(
        "LauncherFar", Commands.runOnce(() -> launcher.startShooter(5000), launcher));

    // Indexer commands
    NamedCommands.registerCommand(
        "IndexerOn", Commands.runOnce(() -> indexer.forwardIndexer(), indexer));
    NamedCommands.registerCommand(
        "IndexerOff", Commands.runOnce(() -> indexer.stopIndexer(), indexer));
    NamedCommands.registerCommand(
        "ToggleIndexer", Commands.runOnce(() -> indexer.toggleIndexer(), indexer));
    
    // Turret commands
    NamedCommands.registerCommand(
        "TurretAutoAimOn", Commands.runOnce(() -> turret.startAutoAim(), turret));
    NamedCommands.registerCommand(
        "TurretAutoAimOff", Commands.runOnce(() -> turret.stopAutoAim(), turret));
    NamedCommands.registerCommand(
        "TurretAutoAimToggle", Commands.runOnce(() -> turret.setAutoAim(!turret.isAutoAimEnabled()), turret));
    NamedCommands.registerCommand(
        "TurretZero", Commands.runOnce(() -> turret.zeroEncoder(), turret).ignoringDisable(true));
    
    // Hood commands
    NamedCommands.registerCommand(
        "HoodEnable", Commands.runOnce(() -> hood.setEnabled(true), hood));
    NamedCommands.registerCommand(
        "HoodDisable", Commands.runOnce(() -> hood.setEnabled(false), hood));
    NamedCommands.registerCommand(
        "HoodSetManual", Commands.runOnce(() -> hood.setManualAngle(15.0), hood));
  }

  private void configureButtonBindings() {
    // Default command - field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° (L1 button)
    controller
        .button(5)
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // X pattern (R1 button)
    controller.button(11).onTrue(Commands.runOnce(drive::stopWithX, drive));
    
    // Toggle Intake (Circle button)
    controller.button(3).onTrue(Commands.runOnce(() -> intake.toggleIntake()));
    
    // Toggle Indexer (Square button)
    controller.button(2).onTrue(Commands.runOnce(() -> indexer.toggleIndexer()));
    
    // Toggle Launcher (Cross button)
    controller.button(1).onTrue(Commands.runOnce(() -> launcher.toggleShooter()));
    
    // Turret AutoAim On (POV Up)
    controller.povUp().onTrue(Commands.runOnce(() -> turret.startAutoAim()));
    
    // Turret AutoAim Off (POV Down)
    controller.povDown().onTrue(Commands.runOnce(() -> turret.stopAutoAim()));
    
    // Toggle Turret AutoAim (Triangle button)
    controller.button(4).onTrue(Commands.runOnce(() -> turret.setAutoAim(!turret.isAutoAimEnabled())));

    // Reset gyro to 0° (L2 button)
    controller
        .button(6)
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void periodic() {
    // Turret'i güncelle
    if (drive != null) {
      turret.update(drive.getPose());
    }
    
    // Launcher ateş izni kontrolü
    if (turret.isReadyToShoot()) {
      launcher.setCanShoot(true);
    } else {
      launcher.setCanShoot(false);
    }
    
    // Tüm subsystem'lerin loglarını kaydet
    Logger.recordOutput("RobotContainer/DrivePose", drive != null ? drive.getPose() : new Pose2d());
    Logger.recordOutput("RobotContainer/Alliance", DriverStation.getAlliance().orElse(Alliance.Blue).toString());
    Logger.recordOutput("RobotContainer/Mode", Constants.currentMode.toString());
  }
}