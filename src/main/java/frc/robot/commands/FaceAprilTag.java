package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight2;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

/**
 * Faces the currently seen AprilTag by driving rotation to reduce Limelight tx to zero.
 * Uses a simple proportional open-loop rotation; translation is held at zero so you only spin.
 *
 * Verbose notes on why:
 * - We only adjust rotational rate because the goal is to aim the robot, not translate toward the tag.
 * - We consume the Limelight's tx (horizontal offset) instead of precomputed yaw so we always chase the live target.
 * - Guarding on tv prevents spinning when the camera has no target; stale data can otherwise whip the robot.
 * - Deadband + tolerance acknowledge that being "close enough" is safer than oscillating around perfect zero.
 * - Output clamping and slew limiting keep angular acceleration gentle so drivers and mechanisms aren't jolted.
 */
public class FaceAprilTag extends Command {

  private final CommandSwerveDrivetrain myDrivetrain;
  private final Limelight2 limelight;
  private final PIDController yawPID;
  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();
  // Slew limiter to smooth rotational rate commands near the target.
  private final edu.wpi.first.math.filter.SlewRateLimiter omegaSlew =
      new edu.wpi.first.math.filter.SlewRateLimiter(Math.PI); // rad/s^2


  public FaceAprilTag(CommandSwerveDrivetrain drivetrain, Limelight2 limelight) {
    myDrivetrain = drivetrain;
    this.limelight = limelight;
    // PID tuned intentionally gentle: lower P/D, zero I to avoid windup when target is lost.
    yawPID = new PIDController(0.9, 0.0, 0.10);
    yawPID.setSetpoint(0.0); // zero tx => aimed at tag
    yawPID.setTolerance(3.0); // small margin of error reduces hunting and driver surprise
    addRequirements(drivetrain, limelight);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    yawPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If no target, hold still to avoid spinning on stale data.
    if (Limelight2.getTV() == 0) {
      yawPID.reset(); // clear accumulated error when target is lost
      myDrivetrain.setControl(
          driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
      return;
    }

    // Small deadband on tx to prevent micro-oscillation; PID still handles outside tolerance.
    double txDeg = MathUtil.applyDeadband(Limelight2.getTxDegrees(), 0.5);
    double omegaDeg = yawPID.calculate(txDeg);
    // Clamp to a gentle max and slew-limit for smooth approach; protects hardware and driver comfort.
    double omegaRad = Units.degreesToRadians(MathUtil.clamp(omegaDeg, -60.0, 60.0));
    omegaRad = omegaSlew.calculate(omegaRad);

    myDrivetrain.setControl(
        driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(omegaRad));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myDrivetrain.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Do not finish if no valid target; otherwise stop when within tolerance.
    return Limelight2.getTV() != 0 && yawPID.atSetpoint();
  }
}
