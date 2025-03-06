// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.extend.Extend;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.util.SetpointConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreCoral extends Command {
  Pivot pivot;
  Extend extend;
  double prepAngleDegrees;
  double prepLengthInches;

  /** Creates a new ScoreCoral. */
  public ScoreCoral(Pivot pivot, Extend extend) {
    pivot = this.pivot;
    extend = this.extend;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot, extend);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    prepAngleDegrees = pivot.getAngle().getDegrees();
    prepLengthInches = extend.getLength(prepAngleDegrees);
    new ArmToSetPoint(pivot, extend, prepAngleDegrees + 5, prepLengthInches - 5, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new ArmToSetPoint(
        pivot,
        extend,
        SetpointConstants.HOME.pivotAngleDegrees(),
        SetpointConstants.HOME.extendLengthInches(),
        true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
