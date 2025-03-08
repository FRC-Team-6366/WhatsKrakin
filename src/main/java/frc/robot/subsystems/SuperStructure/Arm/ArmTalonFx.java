// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure.Arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants;

/** Add your docs here. */
public class ArmTalonFx implements ArmIO {

  private final TalonFX _angleMotorK;
  private final CANcoder _angleCANcoder;
  private double setPointAngleRotations = 0;

  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          SuperStructureConstants.AngleS,
          SuperStructureConstants.AngleG,
          SuperStructureConstants.AngleV);

  private final StatusSignal<Angle> absolutePosition;
  private final StatusSignal<AngularVelocity> absoluteVelocity;
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> supplyCurrentAmps;
  private final StatusSignal<Current> torqueCurrentAmps;
  private final StatusSignal<Temperature> tempCelsius;

  private final PositionVoltage positonOut = new PositionVoltage(0).withSlot(0);
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0);

  public ArmTalonFx() {
    _angleMotorK = new TalonFX(SuperStructureConstants.ArmId);
    _angleCANcoder = new CANcoder(SuperStructureConstants.ArmEncoderId);
    position = _angleMotorK.getPosition();
    velocity = _angleMotorK.getVelocity();
    voltage = _angleMotorK.getMotorVoltage();
    supplyCurrentAmps = _angleMotorK.getSupplyCurrent();
    torqueCurrentAmps = _angleMotorK.getTorqueCurrent();
    tempCelsius = _angleMotorK.getDeviceTemp();
    absolutePosition = _angleCANcoder.getAbsolutePosition();
    absoluteVelocity = _angleCANcoder.getVelocity();

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    // spotless:off
    cfg.MotorOutput
        .withInverted(SuperStructureConstants.ArmInvert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);
    cfg.CurrentLimits
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(40);
    cfg.ClosedLoopGeneral.ContinuousWrap = true; //true = knows when it reaches 360, it is 0
    cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
    cfg.Slot0.kP =  SuperStructureConstants.AngleP;
    cfg.Slot0.kI = SuperStructureConstants.AngleI;
    cfg.Slot0.kD = SuperStructureConstants.AngleD;
    // cfg.Slot0.kG = SuperStructureConstants.AngleG;
    // cfg.Slot0.kV = SuperStructureConstants.AngleV;
    // cfg.Slot0.kS = SuperStructureConstants.AngleS;
    // cfg.Slot0.kA = SuperStructureConstants.AngleA;
    cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = SuperStructureConstants.angleSoftLimitHigh;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = SuperStructureConstants.angleSoftLimitLow;
    cfg.Feedback.SensorToMechanismRatio = 1;
    cfg.Feedback.RotorToSensorRatio = SuperStructureConstants.angleGearRatio;
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    cfg.Feedback.FeedbackRemoteSensorID = _angleCANcoder.getDeviceID(); //connecting CAN to motor
    // voltage limits
    cfg.Voltage.PeakForwardVoltage = SuperStructureConstants.anglePeakVoltage;
    cfg.Voltage.PeakReverseVoltage = -SuperStructureConstants.anglePeakVoltage;
    // spotless:on

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        position,
        velocity,
        voltage,
        supplyCurrentAmps,
        torqueCurrentAmps,
        tempCelsius,
        absolutePosition,
        absoluteVelocity);
    _angleMotorK.optimizeBusUtilization(0.0, 1.0);
    _angleCANcoder.optimizeBusUtilization(0.0, 1.0);

    _angleMotorK.getConfigurator().apply(cfg);
  }

  @Override
  public void setAngle(double angle) {
    double ff =
        feedforward.calculate(
            _angleMotorK.getPosition().getValueAsDouble(),
            _angleMotorK.getVelocity().getValueAsDouble());

    if ((angle * 360 < SuperStructureConstants.PrepAngle
            && _angleMotorK.getPosition().getValueAsDouble() * 360
                > SuperStructureConstants.PrepAngle + 2)
        || (angle * 360 > SuperStructureConstants.PrepAngle
            && _angleMotorK.getPosition().getValueAsDouble() * 360
                < SuperStructureConstants.PrepAngle
                    - 2)) { // protect against rotating under into the wall
      setPointAngleRotations =
          Units.degreesToRotations(
              SuperStructureConstants
                  .PrepAngle); // if arm is arm below 90, go to 90. if arm is arm above 90, go to
      // 90.
    } else {
      setPointAngleRotations = angle;
    }
    _angleMotorK.setControl(
        positonOut.withPosition(setPointAngleRotations).withSlot(0).withFeedForward(ff));
  }

  @Override
  public Rotation2d getAngle() {
    return new Rotation2d(Units.rotationsToRadians(_angleMotorK.getPosition().getValueAsDouble()));
  }

  @Override
  public void runVolts(double volts) {
    _angleMotorK.setControl(voltageOut.withOutput(volts));
  }

  public void toggleBrake(boolean brakeMode) {
    _angleMotorK.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                position,
                velocity,
                voltage,
                supplyCurrentAmps,
                torqueCurrentAmps,
                tempCelsius,
                absolutePosition,
                absoluteVelocity)
            .isOK();

    inputs.positionAngle = Units.rotationsToDegrees(position.getValueAsDouble());
    inputs.velocityRPM = Units.radiansPerSecondToRotationsPerMinute(velocity.getValueAsDouble());
    inputs.goToAngleDegrees = Units.rotationsToDegrees(setPointAngleRotations);

    inputs.appliedVoltage = voltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrentAmps.getValueAsDouble();
    inputs.temperatureCelsius =
        tempCelsius.getValueAsDouble(); // dont add absolute position or coder stuff

    positonOut.FeedForward =
        feedforward.calculate(
            _angleMotorK.getPosition().getValueAsDouble(),
            _angleMotorK.getVelocity().getValueAsDouble());
    // TODO add Input logging
  }
}
