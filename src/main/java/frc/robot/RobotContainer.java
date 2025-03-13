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
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.Auto.ArmL4;
import frc.robot.commands.Auto.ArmPrep;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SuperStructure.Arm.Arm;
import frc.robot.subsystems.SuperStructure.Arm.ArmSim;
import frc.robot.subsystems.SuperStructure.Arm.ArmTalonFx;
import frc.robot.subsystems.SuperStructure.Climb.Climb;
import frc.robot.subsystems.SuperStructure.Climb.ClimbTalonFX;
import frc.robot.subsystems.SuperStructure.Extension.Extension;
import frc.robot.subsystems.SuperStructure.Extension.ExtensionSim;
import frc.robot.subsystems.SuperStructure.Extension.ExtensionTalonFx;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.IO;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  @SuppressWarnings("unused")
  private final Vision vision;

  private final Drive drive;

  public static IO io = new IO(); // creates a version of IO
  public static Extension extension;
  public static Arm arm;
  public static Climb climb;
  public static SuperStructure superStructure;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  //   private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private Command ArmL4 = new ArmL4();
  private Command ArmPrep = new ArmPrep();
  // TODO make collection command

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:

        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1)
                // new VisionIOPhotonVision(camera2Name, robotToCamera2),
                // new VisionIOPhotonVision(camera3Name, robotToCamera3)
                );
        arm = new Arm(new ArmTalonFx());
        extension = new Extension(new ExtensionTalonFx());
        climb = new Climb(new ClimbTalonFX());
        superStructure = new SuperStructure(arm, extension, climb);

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose)
                // new VisionIOPhotonVisionSim(camera2Name, robotToCamera2, drive::getPose),
                // new VisionIOPhotonVisionSim(camera3Name, robotToCamera3, drive::getPose)
                );
        arm = new Arm(new ArmSim());
        extension = new Extension(new ExtensionSim());
        climb = new Climb(new ClimbTalonFX());
        superStructure = new SuperStructure(arm, extension, climb);
        break;

      default:
        // Replayed robot, disable IO implementations

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        superStructure = new SuperStructure(null, null, null);
        break;
    }

    NamedCommands.registerCommand("L4", ArmL4);
    NamedCommands.registerCommand("Prep", ArmPrep);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
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
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    controller
        .leftTrigger(0.1)
        .whileTrue(
            Commands.runOnce(
                () -> {
                  Pose2d currentPose = drive.getPose();

                  Pose2d startPos =
                      new Pose2d(currentPose.getTranslation(), currentPose.getRotation());
                  Pose2d endPos = new Pose2d(3.65, 2.77, new Rotation2d(240));

                  List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
                  PathPlannerPath path =
                      new PathPlannerPath(
                          waypoints,
                          new PathConstraints(
                              1, 1, Units.degreesToRadians(360), Units.degreesToRadians(540)),
                          null,
                          new GoalEndState(0.0, endPos.getRotation()));

                  path.preventFlipping = true;

                  AutoBuilder.followPath(path).schedule();
                }));

    controller
        .rightTrigger(0.1)
        .whileTrue(
            Commands.runOnce(
                () -> {
                  Pose2d currentPose = drive.getPose();

                  Pose2d startPos =
                      new Pose2d(currentPose.getTranslation(), currentPose.getRotation());
                  Pose2d endPos = new Pose2d(3.77, 2.61, new Rotation2d(240));

                  List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
                  PathPlannerPath path =
                      new PathPlannerPath(
                          waypoints,
                          new PathConstraints(
                              1, 1, Units.degreesToRadians(360), Units.degreesToRadians(540)),
                          null,
                          new GoalEndState(0.0, endPos.getRotation()));

                  path.preventFlipping = true;

                  AutoBuilder.followPath(path).schedule();
                }));
  }

  // private void configurePoleBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
