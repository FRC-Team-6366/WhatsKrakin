// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.extend.Extend;
import frc.robot.subsystems.pivot.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ExtendToLength extends Command {
  Pivot pivot;
  Extend extend;
  double extendLengthInch;
  double currentPivotRotations;

  /** Creates a new ExtendToLength. */
  public ExtendToLength(Pivot param_pivot, Extend param_extend, double param_extendLengthInch) {
    pivot = param_pivot;
    extend = param_extend;
    extendLengthInch = param_extendLengthInch;
    addRequirements(extend);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivot.setBrakeMode(true); // set pivot to brake while extending
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPivotRotations = pivot.getAngle().getRotations();
    extend.extendToLength(extendLengthInch, currentPivotRotations);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.setBrakeMode(false); // return pivot to coast when done extending
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return extend.atSetPoint(currentPivotRotations);
  }
}
