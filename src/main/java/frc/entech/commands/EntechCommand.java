/*
 * To change this license header, choose License Headers in Project Properties. To change this
 * template file, choose Tools | Templates and open the template in the editor.
 */
package frc.entech.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.entech.subsystems.EntechSubsystem;

public class EntechCommand extends Command {
  public static final double DEFAULT_TIMEOUT_SECONDS = 60.0;

  public EntechCommand() {}

  public EntechCommand(EntechSubsystem<?, ?> subsystem) {
    this(subsystem, DEFAULT_TIMEOUT_SECONDS);
  }

  public EntechCommand(EntechSubsystem<?, ?> subsystem1, EntechSubsystem<?, ?> subsystem2) {
    addRequirements(subsystem1, subsystem2);
  }

  public EntechCommand(EntechSubsystem<?, ?> subsystem1, EntechSubsystem<?, ?> subsystem2,
      EntechSubsystem<?, ?> subsystem3) {
    addRequirements(subsystem1, subsystem2, subsystem3);
  }

  public EntechCommand(EntechSubsystem<?, ?> subsystem, double timeout) {
    addRequirements(subsystem);
  }

  public EntechCommand(EntechSubsystem<?, ?>... subsystem) {
    addRequirements(subsystem);
  }
}
