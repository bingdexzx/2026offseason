package frc.robot.subsystems;

public class WantedStatesData {
  public final double shooterPos;
  public final double inatkePos;
  public final double shooterVelocity;
  public final double intakeVol;
  public final double feedVol;
  public final double shooterFeedVel;
  // the speed of intake and two feeders are static

  public WantedStatesData(
      double shooterPos,
      double vel,
      double inatkePos,
      double intakeVol,
      double feedVol,
      double shooterFeedVel) {
    this.inatkePos = inatkePos;
    this.shooterPos = shooterPos;
    this.shooterVelocity = vel;
    this.intakeVol = intakeVol;
    this.feedVol = feedVol;
    this.shooterFeedVel = shooterFeedVel;
  }
  ;
}
