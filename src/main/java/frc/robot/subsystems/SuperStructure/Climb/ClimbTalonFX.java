package frc.robot.subsystems.SuperStructure.Climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
// import frc.robot.subsystems.SuperStructure.Climb.ClimbIO.ClimbIOInputs;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants;

public class ClimbTalonFX implements ClimbIO {
  private final TalonFX _climbMotorK;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> torqueCurrentAmps;
  private final StatusSignal<Temperature> tempCelsius;

  private final PositionVoltage positionOut = new PositionVoltage(0).withSlot(0);
  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true).withUpdateFreqHz(0);

  public ClimbTalonFX() {
    _climbMotorK = new TalonFX(SuperStructureConstants.ClimbId);
    position = _climbMotorK.getPosition();
    velocity = _climbMotorK.getVelocity();
    voltage = _climbMotorK.getMotorVoltage();
    supplyCurrentAmps = _climbMotorK.getSupplyCurrent();
    torqueCurrentAmps = _climbMotorK.getTorqueCurrent();
    tempCelsius = _climbMotorK.getDeviceTemp();

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    // spotless:off
    cfg.MotorOutput.withInverted(
            SuperStructureConstants.ClimbInvert
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
    cfg.CurrentLimits.withStatorCurrentLimitEnable(true).withSupplyCurrentLimit(80);
    cfg.ClosedLoopGeneral.ContinuousWrap = false;
    cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
    // cfg.Slot0.kP = SuperStructureConstants.ClimbP;
    // cfg.Slot0.kI = SuperStructureConstants.ClimbI;
    // cfg.Slot0.kD = SuperStructureConstants.ClimbD;
    // cfg.Slot0.kG = SuperStructureConstants.ClimbG;
    // cfg.Slot0.kV = SuperStructureConstants.ClimbV;
    // cfg.Slot0.kS = SuperStructureConstants.ClimbS;
    // cfg.Slot0.kA = SuperStructureConstants.ClimbA;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = SuperStructureConstants.climbSoftLimitHigh;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = SuperStructureConstants.climbSoftLimitLow;
    cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    // cfg.Feedback.SensorToMechanismRatio = 1; // Does not do anything when using a remote can
    // coder
    cfg.Feedback.RotorToSensorRatio = SuperStructureConstants.climbGearRatio;
    // cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.;

    cfg.Voltage.PeakForwardVoltage = SuperStructureConstants.climbPeakVoltage;
    cfg.Voltage.PeakReverseVoltage = -SuperStructureConstants.climbPeakVoltage;

    _climbMotorK.setPosition(0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, position, velocity, voltage, supplyCurrentAmps, torqueCurrentAmps, tempCelsius);
    _climbMotorK.optimizeBusUtilization(0.0, 1.0);

    _climbMotorK.getConfigurator().apply(cfg);
  }

  @Override
  public void setClimb(double angleClimb) {
    _climbMotorK.setControl(positionOut.withPosition(angleClimb).withSlot(0));
  } // your target should be in rotatiosn

  @Override
  public void runVolts(double volts) {
    _climbMotorK.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public Rotation2d getClimb() {
    return new Rotation2d(Units.rotationsToRadians(_climbMotorK.getPosition().getValueAsDouble()));
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                position, velocity, voltage, supplyCurrentAmps, torqueCurrentAmps, tempCelsius)
            .isOK();

    inputs.positionClimb = Units.rotationsToDegrees(position.getValueAsDouble());
    inputs.velocityRPM = Units.radiansPerSecondToRotationsPerMinute(velocity.getValueAsDouble());

    inputs.appliedVoltage = voltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
    inputs.temperatureCelsius = tempCelsius.getValueAsDouble();
  }
}
