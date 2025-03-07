// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.extend.Extend;
import frc.robot.subsystems.pivot.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmToPoint extends Command {
  Pivot pivot;
  Extend extend;
  double thetaDegrees;
  double rInches;
  double currentPivotRotations;
  boolean saftyExtend;

  /** Creates a new ArmToSetPoint. */
  public ArmToPoint(
      Pivot pivot, Extend extend, double thetaDegrees, double rInches, boolean saftyExtend) {
    // Use addRequirements() here to declare subsystem dependencies.
    pivot = this.pivot;
    extend = this.extend;
    thetaDegrees = this.thetaDegrees;
    rInches = this.rInches;
    saftyExtend = this.saftyExtend;

    addRequirements(pivot, extend);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new PivotToAngle(
        pivot,
        extend,
        thetaDegrees,
        saftyExtend); // pivot to angle first, safe pivot chosen in RobotContainer
    new ExtendToLength(pivot, extend, rInches); // extend to length, brake on pivot included
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
