/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package frc.entech.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class EntechSubsystem<I extends SubsystemInput, R extends SubsystemOutput>
    extends SubsystemBase {

  protected EntechSubsystem() {}

  public abstract void initialize();
  
  public abstract boolean isEnabled();

  public abstract void updateInputs(I input);

  public abstract Command getTestCommand();


  public abstract R toOutputs();

  public R getOutputs() {
    R out = toOutputs();
    out.setCurrentCommand(this.getCurrentCommand() + "");
    out.setDefaultCommand(this.getDefaultCommand() + "");
    return out;
  }

}
