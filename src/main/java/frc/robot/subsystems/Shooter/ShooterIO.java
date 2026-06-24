package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double shooterVelocity = 0.0;
    public double shooterVolts = 0.0;
    public double shooterCurrentAmps = 0.0;
    public double feederVelocity = 0.0;
    public double feederVolts = 0.0;
    public double feederCurrentAmps = 0.0;
    public double shooterPosition = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterVelocity(double velocity) {}

  public default void setFeederVelocity(double velocity) {}

  public default void setShooterPos(double position) {}
}
