// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.drive.DriveInput;
import frc.robot.subsystems.extend.Extend;
import frc.robot.subsystems.extend.ExtendIOTalonFX;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.DriveInputSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements DriveInputSupplier {
  // Subsystems
  private final Vision vision;
  private final Pivot pivot;
  private final Extend extend;
  private final SubsystemManager subsystemManager;


  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(SubsystemManager subsystemManager) {
    this.subsystemManager = subsystemManager;
    switch (Constants.currentMode) {
      case REAL:
        vision =
            new Vision(
                subsystemManager.getDriveSubsystem()::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1)
                // new VisionIOPhotonVision(camera2Name, robotToCamera2),
                // new VisionIOPhotonVision(camera3Name, robotToCamera3)
                );
                pivot = new Pivot(new PivotIOTalonFX());
                extend = new Extend(new ExtendIOTalonFX());

        break;

      case SIM:
        vision =
            new Vision(
                subsystemManager.getDriveSubsystem()::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, subsystemManager.getDriveSubsystem()::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, subsystemManager.getDriveSubsystem()::getPose)
                // new VisionIOPhotonVisionSim(camera2Name, robotToCamera2, drive::getPose),
                // new VisionIOPhotonVisionSim(camera3Name, robotToCamera3, drive::getPose)
                );
                pivot =  null;
                extend = null;
        break;

      default:
        // Replayed robot, disable IO implementations
        vision = new Vision(subsystemManager.getDriveSubsystem()::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        pivot = new Pivot(new PivotIOTalonFX());
        extend = new Extend(new ExtendIOTalonFX());
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    subsystemManager.getDriveSubsystem().setDefaultCommand(
        new DriveCommand(subsystemManager.getDriveSubsystem(), this));

    // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    // // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                    subsystemManager.getDriveSubsystem().setPose(
                            new Pose2d(subsystemManager.getDriveSubsystem().getPose().getTranslation(), new Rotation2d())),
                            subsystemManager.getDriveSubsystem())
                .ignoringDisable(true));







    //             extend.setDefaultCommand(Commands.sequence(Commands.run(() -> extend.moveExtend(-0.5), extend).until(() -> extend.extendAtSetPoint(-0.5))
    // .andThen(Commands.run(() -> pivot.movePivot(0)))));



    //Prep Sequences

    m_operatorController.a().whileTrue(
        Commands.sequence(
            Commands.run(() -> pivot.movePivot(0.547))
                .until(() -> pivot.pivotAtSetPoint(0.547))
                .andThen(Commands.run(() -> extend.moveExtend(-0.3)))//L3
        )
    ).onFalse(
            Commands.sequence(
                Commands.run(() -> extend.moveExtend(0.21))
                    .until(() -> extend.extendAtSetPoint(0.21))
                    .andThen(Commands.run(() -> pivot.movePivot(0)))
            )
      );

    // m_operatorController.b().whileTrue(
    //     Commands.sequence(
    //         Commands.run(() -> pivot.movePivot(0.523))
    //             .until(() -> pivot.pivotAtSetPoint(0.523))
    //             .andThen(Commands.run(() -> extend.moveExtend(0)))//L4
    //             )
    // ).onFalse(
    //     Commands.sequence(
    //         Commands.run(() -> extend.moveExtend(-0.5))
    //             .until(() -> extend.extendAtSetPoint(-0.5))
    //             .andThen(Commands.run(() -> pivot.movePivot(0)))
    //         )
    // );

    // m_operatorController.y().whileTrue(
    //     Commands.sequence(
    //         Commands.run(() -> pivot.movePivot(0.47))
    //             .until(() -> pivot.pivotAtSetPoint(0.47))
    //             .andThen(Commands.run(() -> extend.moveExtend(1.57))) //L2
    //         )
    // ).onFalse(
    //     Commands.sequence(Commands.run(() -> extend.moveExtend(-0.5))
    //             .until(() -> extend.extendAtSetPoint(-0.5))
    //             .andThen(Commands.run(() -> pivot.movePivot(0)))
    //     )
    // );

    //Score Sequences

    // m_operatorController.leftBumper().and(m_operatorController.a()).whileTrue(
    //     Commands.sequence(Commands.run(() -> pivot.movePivot(0.7))
    //         .until(() -> pivot.pivotAtSetPoint(0.7))
    //         .andThen(Commands.run(() -> extend.moveExtend(-0.65)))
    //     )
    // );

    //  m_operatorController.leftBumper().and(m_operatorController.b()).whileTrue(
    //     Commands.sequence(Commands.run(() -> pivot.movePivot(0.6))
    //         .until(() -> pivot.pivotAtSetPoint(0.6))
    //         .andThen(Commands.run(() -> extend.moveExtend(-0.7))
    //         .until(() -> extend.extendAtSetPoint(-0.7)))
    //         .andThen(Commands.run(() -> pivot.movePivot(0.45)))
    //     )
    // );

    //  m_operatorController.leftBumper().and(m_operatorController.y()).whileTrue(
    //     Commands.sequence(Commands.run(() -> pivot.movePivot(0.52))
    //         .until(() -> pivot.pivotAtSetPoint(0.52))
    //         .andThen(Commands.run(() -> extend.moveExtend(1)))
    //     )
    // );
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  @Override
  public DriveInput getDriveInput() {
    DriveInput input = new DriveInput();

    input.setGyroAngle(RobotIO.getInstance().getLatesPigeonOutput().getYawPosition());
    input.setLatestOdometryPose(RobotIO.getInstance().getLatestDriveOutput().getEstimatedPose());
    input.setXSpeed(-controller.getLeftY());
    input.setYSpeed(-controller.getLeftX());
    input.setRotation(-controller.getRightX());
    input.setKey("initialRaw");
    return input;
  }
}
