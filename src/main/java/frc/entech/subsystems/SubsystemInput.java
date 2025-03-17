package frc.entech.subsystems;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface SubsystemInput extends LoggableInputs {
  public default String getLogName(String val) {
    return getClass().getSimpleName() + "/" + val;
  }
}
