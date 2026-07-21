package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class FaceCommand extends Command {
  // private final Shooter shooter;
  private final Drive drive;
  private Pose2d currentPose = new Pose2d();
  private DoubleSupplier xSupplier;
  private DoubleSupplier ySupplier;

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.1,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(Units.degreesToRadians(360.0), 5)); // TODO

  private double thetaErrorAbs = 0.0;
  private Rotation2d goalRotation = new Rotation2d();

  public FaceCommand(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    // this.shooter = shooter;
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    // addRequirements(shooter);
    addRequirements(drive);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {

    // =====init target=====//
    // 原点永远定义在蓝方一侧

    if (ifBlue()) {
      goalRotation = new Rotation2d(180);
    } else {
      goalRotation = new Rotation2d(0.0);
    }

    currentPose = drive.getPose();

    ChassisSpeeds fieldVelocity =
        ChassisSpeeds.fromFieldRelativeSpeeds(drive.getChassisSpeeds(), currentPose.getRotation());

    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);

    thetaController.setTolerance(Units.degreesToRadians(1.0));
  }

  @Override
  public void execute() {

    currentPose = drive.getPose();

    // ======================================计算底盘转向速度=============================//
    Rotation2d rotationError = goalRotation.minus(currentPose.getRotation()); // 目标方向-当前方向
    thetaErrorAbs = Math.abs(rotationError.getRadians()); // 弧度误差绝对值

    double thetaFFScaler =
        MathUtil.clamp((Units.radiansToDegrees(thetaErrorAbs) / Math.PI), 0.0, 1.0); // 计算动态前馈参数

    double thetaVelocity =
        thetaController.getSetpoint().velocity * thetaFFScaler
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), goalRotation.getRadians());

    // ========================判断角度达到目标===========================//
    if (thetaErrorAbs < thetaController.getPositionTolerance()) {
      thetaVelocity = 0.0;
    }

    // =========================command drive========================//
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * 0.3,
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * 0.3,
            thetaVelocity,
            currentPose.getRotation()));

    // =======log states=======//

  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), 0.1);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  private boolean ifBlue() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    // shooter.stop();
  }
}
