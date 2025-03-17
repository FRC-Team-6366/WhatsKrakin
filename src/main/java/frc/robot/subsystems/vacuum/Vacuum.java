package frc.robot.subsystems.vacuum;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.vacuum.VacuumIO.VacuumIOInputs;

public class Vacuum extends SubsystemBase{
    private VacuumIO io;
    private VacuumIOInputs inputs = new VacuumIOInputsAutoLogged();

    public Vacuum(VacuumIO io) {
      this.io = io;
    }
  
  
      public void setServoPosition(double angle) {
        io.setServoPosition(angle);
    }

    public void runVolts(double volts) {
        io.runVolts(volts);
    }
  

    /** updates arm values periodically */
    public void periodic() {
      io.updateInputs(inputs);
      // Logger.processInputs("Arm/Vacuum", inputs);
    }

}
