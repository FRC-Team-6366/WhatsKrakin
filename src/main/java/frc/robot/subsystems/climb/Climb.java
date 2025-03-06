package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  // Gear Ratio 180/1

  private ClimbIO io;
  private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged(); /*updates IO inputs
 /* and creates a place to update them */

  public Climb(ClimbIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
  }

  public void runVolts(double volts) {
    io.runVolts(volts);
  }

  public double getAngle() {
    return io.getAngle();
  }
}
