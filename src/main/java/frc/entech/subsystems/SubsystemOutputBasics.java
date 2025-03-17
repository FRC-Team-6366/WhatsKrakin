package frc.entech.subsystems;

import org.littletonrobotics.junction.Logger;

public abstract class SubsystemOutputBasics extends SubsystemOutput {
  protected boolean active;
  protected double currentSpeed;
  protected boolean brakeModeEnabled;
  protected final String key;

  protected SubsystemOutputBasics() {
    this.key = this.getClass().getSimpleName();
  }

  public boolean isActive() {
    return this.active;
  }

  public void setActive(boolean active) {
    this.active = active;
  }

  public double getCurrentSpeed() {
    return this.currentSpeed;
  }

  public void setCurrentSpeed(double currentSpeed) {
    this.currentSpeed = currentSpeed;
  }

  public boolean isBrakeModeEnabled() {
    return this.brakeModeEnabled;
  }

  public void setBrakeModeEnabled(boolean brakeModeEnabled) {
    this.brakeModeEnabled = brakeModeEnabled;
  }

  @Override
  public void toLog() {
    Logger.recordOutput(key + "/active", active);
    Logger.recordOutput(key + "/currentSpeed", currentSpeed);
    Logger.recordOutput(key + "/brakeModeEnabled", brakeModeEnabled);
  }
}
