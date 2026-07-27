package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake.Intake;

public class intakeCommands {
  public static Command intakeUp(Intake intake) {
    return Commands.startEnd(() -> intake.setPos(() -> 0.34), () -> intake.setPos(() -> 0.0));
  }
}
