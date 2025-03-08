// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SuperStructure.Arm.Arm;
import frc.robot.subsystems.SuperStructure.Climb.Climb;
import frc.robot.subsystems.SuperStructure.Extension.Extension;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.otherSequence;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.sequence;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class SuperStructure {
  private Arm arm; // gets arm class as variable and allows for use of arm functions
  private Extension
      extension; // gets extend class as variable and allows for use of extend functions
  private Climb climb;

  private double climbAngle;
  private double armAngle; // arm angle number
  private double extendDistance; // extend distance number
  private String climbPose;
  private String lastPose; // adds a string(a sequence of characters)
  public Pose3d extensionPose =
      new Pose3d(); // creates a pose in 3D by using all the axis (x, y, z) and calculates it
  public Pose3d armPose = new Pose3d(); //

  public SuperStructure(Arm arm, Extension extension, Climb climb) {
    this.arm = arm; // tells arm that it is arm and allows it to be modified
    this.extension = extension;
    this.climb = climb;

    climbPose = otherSequence.ClimbDOne;
    lastPose = sequence.Home; // Makes it go to home no matter the last pose
    armAngle = 0; // logs the arm angle
    extendDistance = 0; // logs the extend distance
    climbAngle = 0; // logs climb angle
  }

  /** updates superstructue values periodically */
  public void structPeriodic() {
    arm.armPeriodic();
    extension.extensionPeriodic();
    climb.climbPeriodic();

    // Im guessing these are SIM
    extensionPose =
        new Pose3d(
            Units.inchesToMeters(-extension.getExtinsion() * arm.getAngle().getCos())
                + Units.inchesToMeters(8),
            0,
            Units.inchesToMeters(extension.getExtinsion() * arm.getAngle().getSin())
                + Units.inchesToMeters(32),
            new Rotation3d(0, arm.getAngle().getRadians() + Units.degreesToRadians(180), 0));
    armPose =
        new Pose3d(
            Units.inchesToMeters(8),
            0,
            Units.inchesToMeters(32),
            new Rotation3d(0, arm.getAngle().getRadians() + Units.degreesToRadians(90), 0));

    Logger.recordOutput("Arm/sequencePose", lastPose);
    Logger.recordOutput("Arm/Pivot/target", armAngle);
    Logger.recordOutput("Arm/Extension/taget", extendDistance);
    Logger.recordOutput("Arm/Extension/pose", extensionPose);
    Logger.recordOutput("Arm/Pivot/pose", armPose);
    Logger.recordOutput("Arm/Pivot/AtTarget", arm.atTarget()); // logs target in advantagescope
    Logger.recordOutput("Climb/ClimbPose", climbPose);
    Logger.recordOutput("Climb/Climb", climbAngle);
    // climb.climbPeriodic(Volts);
  }

  /**
   * @param opA
   * @param opB
   * @param opY
   * @param opX
   * @param drRt
   * @param drLt
   *     <p>Runs the arm in simple teleop control mode (very dumb do not use outside extreme
   *     circumstance)
   */
  public void armTeleop(
      boolean opA, boolean opB, boolean opY, boolean opX, boolean drRt, double drLt) {
    if (opA) {
      armAngle = SuperStructureConstants.L1Angle;
      extendDistance = SuperStructureConstants.L1Extend;
    } else if (opX) {
      armAngle = SuperStructureConstants.L2Angle;
      extendDistance = SuperStructureConstants.L2Extend;
    } else if (opB) {
      armAngle = SuperStructureConstants.L3Angle;
      extendDistance = SuperStructureConstants.L3Extend;
    } else if (opY) {
      armAngle = SuperStructureConstants.L4Angle;
      extendDistance = SuperStructureConstants.L4Extend;
    } else if (drLt > 0.1) {
      armAngle = SuperStructureConstants.CollectPrepAngle;
      extendDistance = SuperStructureConstants.CollectPrepExtend;
    } else if (drLt > 0.75) {
      armAngle = SuperStructureConstants.CollectAngle;
      extendDistance = SuperStructureConstants.CollectExtend;
    }

    if (drRt && drLt < 0.1) { // drop angle to score
      armAngle -= SuperStructureConstants.scoreAngleDrop;
    }

    // update target positions
    arm.setPosition(new Rotation2d(Units.degreesToRadians(armAngle)));

    extension.extendToDistance(extendDistance);
  }

  /**
   * @param opA
   * @param opB
   * @param opY
   * @param opX
   * @param drRt
   * @param drLt runs the arm in a sequence prep -> extend and angle -> score -> retract -> stow
   */
  public void advancedArmTeleop(
      boolean opA,
      boolean opB,
      boolean opY,
      boolean opX,
      boolean drRt,
      double drLt,
      boolean opLb,
      boolean opRb,
      boolean DrY,
      boolean DrRb,
      boolean DrLb,
      boolean DrA
      // boolean opA,
      // boolean opB,
      // boolean opY,
      // boolean opX,
      // boolean drRt,
      // double drLt,
      // boolean opLb,
      // boolean opRb,
      // boolean DrRb,
      // boolean DrLb,
      // boolean DrA
      ) {
    if (DrRb && climb.getClimb().getRotations() > -70) {
      // go to set position in constants or prepares for climb
      climbPose = otherSequence.PrepareClimb;
      climb.runVolts(-12);
      // sequence holder for climb
     
    } else if (DrLb && climb.getClimb().getRotations() > -177) {
      // go to set position in constants and does climb
      // climbAngle = SuperStructureConstants.doClimb;
     climbPose = otherSequence.Climbing;
      climb.runVolts(-12);
      if (climb.getClimb().getRotations() < -177){
        climb.runVolts(0);
        climbPose = otherSequence.ClimbDOne;
       }
      // sequence holder for climb
    }
    // } else if (DrA) {

    //   climbAngle = SuperStructureConstants.HomeClimb;

    //   // sequence holder for climb
    //   climbPose = otherSequence.ClimbHome;
    // }

    if ((opLb && opRb)) { // force end sequnce
      // sets position using constants
      armAngle = SuperStructureConstants.HomeAngle;
      extendDistance = SuperStructureConstants.HomeExtend;
      // sequence holder
      lastPose = sequence.Home;

    } else if (lastPose == sequence.Prep
        && arm.atTarget()
        && drRt) { // move from preped to score to score angle
      // sets position using constants
      extendDistance -= SuperStructureConstants.scoreExtendDrop;
      armAngle -= SuperStructureConstants.scoreAngleDrop;
      // sequence holder
      lastPose = sequence.Score;

    } else if (lastPose == sequence.Score
        && arm.atTarget()
        && !drRt) { // retract extension after score
      // sets position using constants
      extendDistance = SuperStructureConstants.HomeExtend;
      lastPose = sequence.Retract;

    } else if (lastPose == sequence.Retract
        && arm.atTarget()
        && extension.atExtension()) { // once extension retracted return arm to home
      // sets position using constants
      extendDistance = SuperStructureConstants.HomeExtend;
      armAngle = SuperStructureConstants.HomeAngle;
      // sequence holder
      lastPose = sequence.Home;

    }
    // Go to prep pose (should be less than 180 from stow but within 90 of final target)
    else if (opA) { // begin L1 sequence
      // sets position using constants
      armAngle = SuperStructureConstants.PrepAngle;
      extendDistance = SuperStructureConstants.PrepExtend;
      // sequence holder
      lastPose = sequence.L1;

    } else if (opX) { // begin l2 sequence
      // sets position using constants
      armAngle = SuperStructureConstants.PrepAngle;
      extendDistance = SuperStructureConstants.PrepExtend;
      // sequence holder
      lastPose = sequence.L2;

    } else if (opB) { // begin l3 sequence
      // sets position using constants
      armAngle = SuperStructureConstants.PrepAngle;
      extendDistance = SuperStructureConstants.PrepExtend;
      // sequence holder
      lastPose = sequence.L3;

    } else if (opY) { // begin l4 sequence
      // sets position using constants
      armAngle = SuperStructureConstants.PrepAngle;
      extendDistance = SuperStructureConstants.PrepExtend;
      // sequence holder
      lastPose = sequence.L4;

    } else if (drLt > 0.75) { // grab the game piece from the ramp
      // sets position using constants
      armAngle = SuperStructureConstants.CollectAngle;
      extendDistance = SuperStructureConstants.CollectExtend;
      // sequence holder
      lastPose = sequence.Home;

    } else if (drLt > 0.1) { // put arm in position to grab from ramp
      // sets position using constants
      armAngle = SuperStructureConstants.CollectPrepAngle;
      extendDistance = SuperStructureConstants.CollectPrepExtend;
      // sequence holder
      lastPose = sequence.Home;

    }
    // actuall scoring pose to move to after releasing
    else if (!opA
        && lastPose == sequence.L1
        && arm.atTarget()) { // move to final location to score l2
      // sets position using constants
      armAngle = SuperStructureConstants.L1Angle;
      extendDistance = SuperStructureConstants.L1Extend;
      // sequence holder
      lastPose = sequence.Prep;

    } else if (!opB
        && lastPose == sequence.L3
        && arm.atTarget()) { // move to final location to score l3
      // sets position using constants
      armAngle = SuperStructureConstants.L3Angle;
      extendDistance = SuperStructureConstants.L3Extend;
      // sequence holder
      lastPose = sequence.Prep;

    } else if (!opX
        && lastPose == sequence.L2
        && arm.atTarget()) { // move to final location to score l2
      // sets position using constants
      armAngle = SuperStructureConstants.L2Angle;
      extendDistance = SuperStructureConstants.L2Extend;
      // sequence holder
      lastPose = sequence.Prep;
    } else if (!opY
        && lastPose == sequence.L4
        && arm.atTarget()) { // move to final locaiton to score l4
      // sets position using constants
      armAngle = SuperStructureConstants.L4Angle;
      extendDistance = SuperStructureConstants.L4Extend;

      // sequence holder
      lastPose = sequence.Prep;
    }
    // Home Pose
    else if (lastPose == sequence.Home || (opLb && opRb)) { // go to home if not in sequence
      // sets position using constants
      armAngle = SuperStructureConstants.HomeAngle;
      extendDistance = SuperStructureConstants.HomeExtend;
      // sequence holder
      lastPose = sequence.Home;
    }

    arm.setPosition(new Rotation2d(Units.degreesToRadians(armAngle)));
    //  climb.setClimbPosition(new Rotation2d(Units.degreesToRadians(climbAngle)));
    extension.extendToDistance(extendDistance);
  }

  public void setArm(double angle) {
    arm.setPosition(new Rotation2d(Units.degreesToRadians(angle)));
    armAngle = angle;
  }

  public void setExtension(double extend) {
    extension.extendToDistance(extend);
    extendDistance = extend;
  }

  // public void setClimb(double angle){
  //   climb.setClimbPosition(angle);
  //   climbAngle = angle;
  // }

  public boolean atSetpoint() {
    return arm.atTarget() && extension.atExtension();
  }
}
