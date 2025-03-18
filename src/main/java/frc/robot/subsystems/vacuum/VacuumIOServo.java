package frc.robot.subsystems.vacuum;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Servo;

public class VacuumIOServo implements VacuumIO {
    private final Servo _servoMotor;
    private final TalonSRX _talonSRX;

    public VacuumIOServo() {
        _servoMotor = new Servo(0);

        _talonSRX = new TalonSRX(30);
        _talonSRX.configFactoryDefault();
        _talonSRX.setInverted(false);   
        // _talonSRX.set(ControlMode.PercentOutput, 0);
       
        /* Factory Default all hardware to prevent unexpected behaviour */
		/* Set neutral modes */
		// _talonSRX.setNeutralMode(NeutralMode.Brake);
        // _talonSRX.configPeakOutputForward(+1.0, 30);
		// _talonSRX.configPeakOutputReverse(-1.0, 30);


    }

    @Override
    public void updateInputs(VacuumIOInputs inputs) {
        inputs.servoPositionAngle = _servoMotor.getAngle();
    }

    @Override
    public void runVolts(double volts) {
        _talonSRX.set(TalonSRXControlMode.PercentOutput, volts);
        // System.out.println("Output:" + volts);
    }

    @Override
    public void setServoPosition(double angle) {
        _servoMotor.setAngle(angle);
    }
    
}
