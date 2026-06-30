package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;

public class SuperStructure extends SubsystemBase {
  private Shooter shooter;
  private Intake intake;

  public SuperStructure(Shooter shooter, Intake intake) {
    this.intake = intake;
    this.shooter = shooter;
  }

  public void priodic() {}
}
