package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight2;
import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * Faces the currently seen AprilTag by driving rotation to reduce Limelight tx to zero.
 * Uses a simple proportional open-loop rotation; translation is held at zero so you only spin.
 */
public class FaceAprilTag extends Command {
  private static final double kP = 0.04; // rad error -> normalized output; tune to preference
  private static final double kMaxRotRadPerSec = Math.toRadians(2.5); // cap spin rate
  private static final double kDeadbandRad = Math.toRadians(1.0); // stop jitter when nearly centered

  private final CommandSwerveDrivetrain drivetrain;
  private final Limelight2 lime;

  private final SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric();
  double radPerS = 0.0;

  public FaceAprilTag(CommandSwerveDrivetrain drive, Limelight2 limelight) {
    drivetrain = drive;
    lime = limelight;
    addRequirements(drivetrain, lime);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Start with everything zeroed so the robot sits still before first measurement.
    drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double omega = 0.0;
    if (Limelight2.hasTarget()) {
      // Positive tx means target is to the right; spin toward center using a simple P loop.
      double error = Limelight2.getTxRadians();
      if (Math.abs(error) > kDeadbandRad) {
        omega = MathUtil.clamp(error * kP, -1.0, 1.0) * kMaxRotRadPerSec;
      }
    }
    drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(omega));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop rotation when released or interrupted so we do not keep spinning.
    drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
