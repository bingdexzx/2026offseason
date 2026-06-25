package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX shotMotor;
  private final TalonFX feedMotor_2;
  private final TalonFX feedMotor_1;
  private final TalonFX turnMotor;

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // Motion Magic requests
  private final MotionMagicVelocityVoltage speedMMrequest = new MotionMagicVelocityVoltage(0.0);
  private final MotionMagicVoltage turnMMreuqest = new MotionMagicVoltage(0.0);

  public ShooterIOTalonFX() {
    TalonFXConfiguration shotMotorConfig = new TalonFXConfiguration();
    TalonFXConfiguration feedMotorConfig = new TalonFXConfiguration();
    TalonFXConfiguration turnMotorConfig = new TalonFXConfiguration();
    //
    shotMotorConfig.MotionMagic.MotionMagicAcceleration = 50;
    shotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 50;
    shotMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    shotMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    shotMotorConfig.Voltage.PeakForwardVoltage = 12.0;
    shotMotorConfig.Voltage.PeakReverseVoltage = -12.0;
    shotMotorConfig.Slot0.kS = 0.22;
    shotMotorConfig.Slot0.kV = 0.115;
    shotMotorConfig.Slot0.kA = 0.0;
    shotMotorConfig.Slot0.kP = 0.13;
    shotMotorConfig.Slot0.kD = 0.0;
    shotMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    //
    turnMotorConfig.MotionMagic.MotionMagicAcceleration = 5;
    turnMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 3;
    turnMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    turnMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1;
    turnMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    turnMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.3;
    turnMotorConfig.Voltage.PeakForwardVoltage = 12.0;
    turnMotorConfig.Voltage.PeakReverseVoltage = -12.0;
    turnMotorConfig.Slot0.kS = 0.0;
    turnMotorConfig.Slot0.kV = 0.0;
    turnMotorConfig.Slot0.kG = 0.0;
    turnMotorConfig.Slot0.kA = 0.0;
    turnMotorConfig.Slot0.kP = 0.0;
    turnMotorConfig.Slot0.kD = 0.0;
    turnMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    turnMotorConfig.Feedback.SensorToMechanismRatio = 20.0;
    //
    feedMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    feedMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    feedMotorConfig.Voltage.PeakForwardVoltage = 12.0;
    feedMotorConfig.Voltage.PeakReverseVoltage = -12.0;
    feedMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    this.shotMotor = new TalonFX(15);
    this.feedMotor_2 = new TalonFX(16); // shooter上的滚筒
    this.feedMotor_1 = new TalonFX(17); // feeder_1 底盘上的多组皮带
    this.turnMotor = new TalonFX(18);

    turnMotor.getConfigurator().apply(turnMotorConfig);
    shotMotor.getConfigurator().apply(shotMotorConfig);
    feedMotor_1.getConfigurator().apply(feedMotorConfig);
    feedMotor_2.getConfigurator().apply(feedMotorConfig);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterVelocity = shotMotor.getVelocity().getValueAsDouble();
    inputs.shooterCurrentAmps = shotMotor.getSupplyCurrent().getValueAsDouble();
    inputs.feederVelocity = feedMotor_2.getVelocity().getValueAsDouble();
    inputs.shooterPosition = turnMotor.getPosition().getValueAsDouble();
    inputs.shotVelocitySetPoint = shotMotor.getClosedLoopReference().getValueAsDouble();
    inputs.feedVelSetpoint = feedMotor_2.getClosedLoopReference().getValueAsDouble();
  }

  @Override
  public void setShooterVelocity(double velocity) {
    shotMotor.setControl(speedMMrequest.withVelocity(velocity));
  }

  @Override
  public void setShooterPos(double pos) {
    turnMotor.setControl(turnMMreuqest.withPosition(pos));
  }

  @Override
  public void setFeeder_1Vol(double vol) {
    feedMotor_1.setControl(voltageRequest.withOutput(vol));
  }

  @Override
  public void setFeeder_2Velocity(double velocity) {
    feedMotor_2.setControl(speedMMrequest.withVelocity(velocity));
  }
}
