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
  private final TalonFX feedMorter;
  private final TalonFX turnMortor;

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
    shotMotorConfig.MotionMagic.MotionMagicAcceleration = 20;
    shotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 10;
    shotMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    shotMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    shotMotorConfig.Voltage.PeakForwardVoltage = 12.0;
    shotMotorConfig.Voltage.PeakReverseVoltage = -12.0;
    shotMotorConfig.Slot0.kS = 0.22;
    shotMotorConfig.Slot0.kV = 0.0;
    shotMotorConfig.Slot0.kA = 0.0;
    shotMotorConfig.Slot0.kP = 0.0;
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
    //
    feedMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    feedMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    feedMotorConfig.Voltage.PeakForwardVoltage = 12.0;
    feedMotorConfig.Voltage.PeakReverseVoltage = -12.0;
    feedMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    this.shotMotor = new TalonFX(15);
    this.feedMorter = new TalonFX(16);
    this.turnMortor = new TalonFX(18);

    turnMortor.getConfigurator().apply(turnMotorConfig);
    shotMotor.getConfigurator().apply(shotMotorConfig);
    feedMorter.getConfigurator().apply(feedMotorConfig);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterVelocity = shotMotor.getVelocity().getValueAsDouble();
    inputs.shooterVolts = shotMotor.getMotorVoltage().getValueAsDouble();
    inputs.shooterCurrentAmps = shotMotor.getSupplyCurrent().getValueAsDouble();
    inputs.feederVelocity = feedMorter.getVelocity().getValueAsDouble();
    inputs.shooterPosition = shotMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void setShooterVelocity(double velocity) {
    shotMotor.setControl(speedMMrequest.withVelocity(velocity));
  }

  @Override
  public void setShooterPos(double pos) {
    turnMortor.setControl(turnMMreuqest.withPosition(pos));
  }
}
