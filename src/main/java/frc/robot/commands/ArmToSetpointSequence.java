// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.extend.Extend;
import frc.robot.subsystems.pivot.Pivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmToSetpointSequence extends SequentialCommandGroup {
  /** Creates a new ArmToSetpointSequence. */
  public ArmToSetpointSequence(
      Pivot pivot, Extend extend, double thetaDegrees, double rInches, boolean saftyExtend) {

    addCommands(
        new PivotToAngle(pivot, extend, thetaDegrees, saftyExtend),
        new ExtendToLength(pivot, extend, rInches));
    addRequirements(pivot, extend);
  }
}
