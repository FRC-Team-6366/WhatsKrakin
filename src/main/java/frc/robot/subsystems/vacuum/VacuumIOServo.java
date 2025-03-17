package frc.robot.subsystems.vacuum;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Servo;

public class VacuumIOServo implements VacuumIO {
    private final Servo _servoMotor;
    private final TalonSRX _talonSRX;

    public VacuumIOServo() {
        _servoMotor = new Servo(0);

        _talonSRX = new TalonSRX(0);
        _talonSRX.setInverted(false);

    }

    @Override
    public void updateInputs(VacuumIOInputs inputs) {
        inputs.servoPositionAngle = _servoMotor.getAngle();
    }

    @Override
    public void runVolts(double volts) {
        _talonSRX.set(ControlMode.PercentOutput, volts);
    }

    @Override
    public void setServoPosition(double angle) {
        _servoMotor.setAngle(angle);
    }
    
}
