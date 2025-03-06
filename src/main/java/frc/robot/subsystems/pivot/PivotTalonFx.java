// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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

/** Add your docs here. */
public class PivotTalonFx implements PivotIO {

  private final TalonFX _pivotMotorK;
  private final CANcoder _pivotCANCoder;

  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocityRotationsPerSecond;
  private final StatusSignal<Angle> absolutePositionRotations;
  private final StatusSignal<AngularVelocity> absoluteVelocityRotationsPerSecond;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> torqueCurrentAmps;
  private final StatusSignal<Temperature> tempCelsius;

  private final PositionVoltage positionOut = new PositionVoltage(0).withSlot(0);
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);

  private double setpointAngleDegrees;

  public PivotTalonFx() {
    _pivotMotorK = new TalonFX(PivotConstants.pivotTalonId);
    _pivotCANCoder = new CANcoder(PivotConstants.pivotCANCoderId);
    positionRotations = _pivotMotorK.getPosition();
    velocityRotationsPerSecond = _pivotMotorK.getVelocity();
    voltage = _pivotMotorK.getMotorVoltage();
    supplyCurrentAmps = _pivotMotorK.getSupplyCurrent();
    torqueCurrentAmps = _pivotMotorK.getTorqueCurrent();
    tempCelsius = _pivotMotorK.getDeviceTemp();
    absolutePositionRotations = _pivotCANCoder.getAbsolutePosition();
    absoluteVelocityRotationsPerSecond = _pivotCANCoder.getVelocity();

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    // spotless:off
    cfg.MotorOutput
        .withInverted(
          PivotConstants.pivotInvert
          ? InvertedValue.Clockwise_Positive
          : InvertedValue.CounterClockwise_Positive
        )
        .withNeutralMode(
          PivotConstants.pivotNeutralModeBrake
          ? NeutralModeValue.Brake
          : NeutralModeValue.Coast
        );
    cfg.CurrentLimits
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40);
    cfg.ClosedLoopGeneral.ContinuousWrap = false; //true = knows when it reaches 360, it is 0
    cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
    cfg.Slot0.kP = PivotConstants.kP;
    cfg.Slot0.kI = PivotConstants.kI;
    cfg.Slot0.kD = PivotConstants.kD;
    cfg.Slot0.kG = PivotConstants.kG;
    cfg.Slot0.kS = PivotConstants.kS;
    cfg.Slot0.kV = PivotConstants.kV;
    cfg.Slot0.kA = PivotConstants.kA;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = PivotConstants.pivotForwardSoftLimitEnabled;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = PivotConstants.pivotForwardSoftLimit;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = PivotConstants.pivotReverseSoftLimitEnabled;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = PivotConstants.pivotReverseSoftLimit;
    cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    cfg.Feedback.SensorToMechanismRatio = 1;
    cfg.Feedback.RotorToSensorRatio = PivotConstants.pivotGearRatio;
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    cfg.Feedback.FeedbackRemoteSensorID = _pivotCANCoder.getDeviceID(); //connecting CAN to motor
    
    // voltage limits
    cfg.Voltage.PeakForwardVoltage = PivotConstants.pivotPeakVoltage;
    cfg.Voltage.PeakReverseVoltage = -PivotConstants.pivotPeakVoltage;

    _pivotMotorK.setPosition(_pivotCANCoder.getAbsolutePosition().getValueAsDouble());
    // spotless:on

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        positionRotations,
        velocityRotationsPerSecond,
        voltage,
        supplyCurrentAmps,
        torqueCurrentAmps,
        tempCelsius,
        absolutePositionRotations,
        absoluteVelocityRotationsPerSecond);

    _pivotMotorK.optimizeBusUtilization(0.0, 1.0);
    _pivotCANCoder.optimizeBusUtilization(0.0, 1.0);

    _pivotMotorK.getConfigurator().apply(cfg);
  }

  @Override
  public void setBrakeMode(boolean brakeMode) {
    _pivotMotorK.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void runVolts(double volts) {
    _pivotMotorK.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void pivotToAngle(double angleDegrees) {
    _pivotMotorK.setControl(
        positionOut.withPosition(Units.degreesToRotations(angleDegrees)).withSlot(0));
    setpointAngleDegrees = angleDegrees;
  }

  @Override
  public Rotation2d getAngle() {
    return new Rotation2d(Units.rotationsToRadians(_pivotMotorK.getPosition().getValueAsDouble()));
  }

  @Override
  public void updateConfig(double extendLengthInches) {
    TalonFXConfigurator configurator = _pivotMotorK.getConfigurator();
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Slot0.kG = PivotConstants.kG * extendLengthInches * PivotConstants.kGExtendFactor;

    configurator.refresh(cfg);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {

    inputs.connected =
        BaseStatusSignal.refreshAll(
                positionRotations,
                velocityRotationsPerSecond,
                voltage,
                supplyCurrentAmps,
                torqueCurrentAmps,
                tempCelsius,
                absolutePositionRotations,
                absoluteVelocityRotationsPerSecond)
            .isOK();

    inputs.positionAngleDegrees = Units.rotationsToDegrees(positionRotations.getValueAsDouble());
    inputs.velocityRotationsPerSecond = velocityRotationsPerSecond.getValueAsDouble();
    inputs.appliedVoltage = voltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
  }
  // TODO add Input logging
}
