package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight2;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Usage: Bind to a button (e.g., driver B). Hold to rotate in place until the Limelight tx is ~0; releasing ends it.
 * Requirements: Limelight2 subsystem must be scheduled so its static getters are fresh; drivetrain must be a CTRE swerve.
 * Behavior:
 * - Zeroes X/Y velocity; only commands rotation.
 * - Stops immediately when no target (tv == 0) to avoid spinning on stale data.
 * - PID on tx (deg) with deadband, tolerance, clamp, and slew to keep rotation smooth.
 * - Finishes after ON_TARGET_CYCLES_REQUIRED consecutive cycles within tolerance.
 */
public class FaceAprilTag extends Command {

  private final CommandSwerveDrivetrain myDrivetrain;
  private final Limelight2 limelight;
  private final PIDController yawPID;
  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();
  private static final double MAX_ROTATE_DEG_PER_SEC = 60.0; // clamp to keep aim smooth; tune as needed
  private static final int ON_TARGET_CYCLES_REQUIRED = 5; // debounce to avoid ending on one good frame
  private int onTargetCycles = 0;
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

  /** Reset controller state each time the command is scheduled. */
  @Override
  public void initialize() {
    yawPID.reset();
    onTargetCycles = 0;
  }

  /** Run every scheduler tick: read tx, compute omega, command rotation-only, and track dwell-on-target. */
  @Override
  public void execute() {
    // If no target, hold still to avoid spinning on stale data.
    if (Limelight2.getTV() == 0) {
      yawPID.reset(); // clear accumulated error when target is lost
      onTargetCycles = 0;
      myDrivetrain.setControl(
          driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
      return;
    }

    // Small deadband on tx to prevent micro-oscillation; PID still handles outside tolerance.
    double txDeg = MathUtil.applyDeadband(Limelight2.getTxDegrees(), 0.5);
    double omegaDeg = yawPID.calculate(txDeg);
    // Clamp to a gentle max and slew-limit for smooth approach; protects hardware and driver comfort.
    double omegaRad = Units.degreesToRadians(MathUtil.clamp(omegaDeg, -MAX_ROTATE_DEG_PER_SEC, MAX_ROTATE_DEG_PER_SEC));
    omegaRad = omegaSlew.calculate(omegaRad);

    myDrivetrain.setControl(
        driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(omegaRad));

    // Live debug telemetry for aiming.
    SmartDashboard.putBoolean("FaceAprilTag/Active", true);
    SmartDashboard.putNumber("FaceAprilTag/TxDegrees", txDeg);
    SmartDashboard.putNumber("FaceAprilTag/OmegaCmdRad", omegaRad);

    // Track on-target dwell to prevent a single good frame from ending the command.
    if (yawPID.atSetpoint()) {
      onTargetCycles++;
    } else {
      onTargetCycles = 0;
    }
  }

  /** On exit (normal or interrupted), ensure we stop rotating. */
  @Override
  public void end(boolean interrupted) {
    myDrivetrain.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    SmartDashboard.putBoolean("FaceAprilTag/Active", false);
    SmartDashboard.putNumber("FaceAprilTag/OnTargetCycles", onTargetCycles);
  }

  /** Finish after the target is held within tolerance for the configured dwell. */
  @Override
  public boolean isFinished() {
    // Do not finish if no valid target; otherwise stop when within tolerance.
    return Limelight2.getTV() != 0 && onTargetCycles >= ON_TARGET_CYCLES_REQUIRED;
  }
}
