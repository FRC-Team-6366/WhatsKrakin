package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.climb.ClimbIO.ClimbIOInputs;

public class ClimbTalonFX implements ClimbIO {

  private final TalonFX _climbMotorK;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> torqueCurrentAmps;
  private final StatusSignal<Temperature> tempCelsius;

  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true).withUpdateFreqHz(0);

  public ClimbTalonFX() {
    _climbMotorK = new TalonFX(ClimbConstants.climbTalonId);
    position = _climbMotorK.getPosition();
    velocity = _climbMotorK.getVelocity();
    voltage = _climbMotorK.getMotorVoltage();
    supplyCurrentAmps = _climbMotorK.getSupplyCurrent();
    torqueCurrentAmps = _climbMotorK.getTorqueCurrent();
    tempCelsius = _climbMotorK.getDeviceTemp();

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    // spotless:off
    cfg.MotorOutput.withInverted(
            ClimbConstants.climbInvert
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(
            ClimbConstants.climbNeutralModeBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    cfg.CurrentLimits.withStatorCurrentLimitEnable(true).withSupplyCurrentLimit(40);
    cfg.ClosedLoopGeneral.ContinuousWrap = false;
    cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = ClimbConstants.climbForwardSoftLimitEnabled;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimbConstants.climbForwardSoftLimit;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = ClimbConstants.climbReverseSoftLimitEnabled;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimbConstants.climbReverseSoftLimit;

    cfg.Voltage.PeakForwardVoltage = ClimbConstants.climbPeakVoltage;
    cfg.Voltage.PeakReverseVoltage = -ClimbConstants.climbPeakVoltage;

    // Set internal encoder to 0 when code starts - climb arm must be manually reset to start
    // position
    _climbMotorK.setPosition(0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, position, velocity, voltage, supplyCurrentAmps, torqueCurrentAmps, tempCelsius);

    _climbMotorK.optimizeBusUtilization(0.0, 1.0);

    _climbMotorK.getConfigurator().apply(cfg);
  }

  @Override
  public void runVolts(double volts) {
    _climbMotorK.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public double getAngle() {
    return position.getValueAsDouble() / ClimbConstants.climbGearRatio;
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                position, velocity, voltage, supplyCurrentAmps, torqueCurrentAmps, tempCelsius)
            .isOK();

    inputs.position = position.getValueAsDouble();
    inputs.angleDegrees =
        Units.rotationsToDegrees(position.getValueAsDouble() / ClimbConstants.climbGearRatio);
    inputs.velocityRPM = Units.radiansPerSecondToRotationsPerMinute(velocity.getValueAsDouble());

    inputs.appliedVoltage = voltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
    inputs.temperatureCelsius = tempCelsius.getValueAsDouble();
  }
}
