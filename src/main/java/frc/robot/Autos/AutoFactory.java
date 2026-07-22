package frc.robot.Autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootCommands;
import frc.robot.commands.intakeCommand;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.drive.Drive;

public class AutoFactory {

  public static Command buildPathCommand(String fileName) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(fileName);
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }

  public static PathPlannerPath getPPPath(String fileName) {
    try {
      return PathPlannerPath.fromPathFile(fileName);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return null;
    }
  }

  public static void setPoseTostartPoint(PathPlannerPath firstPath, Drive drive) {
    drive.setPose(firstPath.getStartingHolonomicPose().get());
  }

  public static Command runSidePathInAuto(
      String AutoName1, String AutoName2, String AutoName3, Intake intake, Shooter shooter) {

    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(new PathPlannerAuto(AutoName1), new intakeCommand(intake)),
        new ParallelDeadlineGroup(new PathPlannerAuto(AutoName2), new intakeCommand(intake)),
        ShootCommands.shootWithTime(shooter, intake, 4),
        new ParallelDeadlineGroup(new PathPlannerAuto(AutoName3), new intakeCommand(intake)),
        new ParallelDeadlineGroup(new PathPlannerAuto(AutoName2), new intakeCommand(intake)),
        ShootCommands.shootWithTime(shooter, intake, 4));
  }

  public static Command runSidePathInAuto(
      String AutoName1,
      String AutoName2,
      String AutoName3,
      String AutoName4,
      Intake intake,
      Shooter shooter) {

    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(new PathPlannerAuto(AutoName1), new intakeCommand(intake)),
        new ParallelDeadlineGroup(new PathPlannerAuto(AutoName2), new intakeCommand(intake)),
        ShootCommands.shootWithTime(shooter, intake, 4),
        new ParallelDeadlineGroup(new PathPlannerAuto(AutoName3), new intakeCommand(intake)),
        new ParallelDeadlineGroup(new PathPlannerAuto(AutoName4), new intakeCommand(intake)));
  }

  public static Command runMiddlePathInAuto(String AutoName1, Intake intake, Shooter shooter) {

    return new SequentialCommandGroup(
        new PathPlannerAuto(AutoName1), ShootCommands.shootWithTime(shooter, intake, 7));
  }
}
