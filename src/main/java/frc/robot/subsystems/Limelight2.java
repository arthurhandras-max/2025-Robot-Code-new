package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

import frc.robot.LimelightHelpers;

//y is up and down, z is forward and backward, x is left and right

public class Limelight2 extends SubsystemBase{
    NetworkTable table2;
    static double x, y, area, distX, distY, distZ, angleTargetRadians, v;
    NetworkTableEntry tx, ty, ta, tv;
    Pose3d targetPose, botPose;
    Rotation3d targetRotation;
    
    public Limelight2(){
        table2 = NetworkTableInstance.getDefault().getTable("limelight-lime");
        tx = table2.getEntry("tx");
        ty = table2.getEntry("ty");
        ta = table2.getEntry("ta");
        tv = table2.getEntry("tv");
    }

    public void updateValues(){
        //read values periodically
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        v = tv.getDouble(0.0);

        targetPose = LimelightHelpers.getTargetPose3d_RobotSpace("limelight-lime");

        distX = targetPose.getX();
        distY = targetPose.getY();
        distZ = targetPose.getZ();

        targetRotation = targetPose.getRotation();
        angleTargetRadians = targetRotation.getZ();
    }
    public void updateDashboard(){
        //post to smart dashboard periodically
        SmartDashboard.putNumber("Limelight2/XDegrees", x);
        SmartDashboard.putNumber("Limelight2/YDegrees", y);
        SmartDashboard.putNumber("Limelight2/YDegrees", v);
        SmartDashboard.putNumber("Limelight2/Area", area);
        SmartDashboard.putNumber("Limelight2/DistanceX", distX);
        SmartDashboard.putNumber("Limelight2/DistanceY", distY);
        SmartDashboard.putNumber("Limelight2/DistanceZ", distZ);
        SmartDashboard.putNumber("Limelight2/TargetYawRadians", angleTargetRadians);
    }
    @Override
    public void periodic(){
        updateValues();
        updateDashboard();
    }

    public static double getDistZ(){
        return distZ;
    }

    /** True when Limelight reports a valid target (tv > 0). */
    public static boolean hasTarget() {
        return v > 0.5;
    }

    /** Latest horizontal angle error (tx) in radians; positive means target is to the right. */
    public static double getTxRadians() {
        return Math.toRadians(x);
    }

    /** Latest detected fiducial ID (-1 if none or unknown). */
    public static int getFiducialId() {
        return (int) LimelightHelpers.getFiducialID("limelight-lime");
    }

    /** True only when a target is present and matches the requested ID. */
    public static boolean hasTargetWithId(int desiredId) {
        return hasTarget() && getFiducialId() == desiredId;
    }

    public static double getAngleTargetRadians(){
        return angleTargetRadians;
    }
}
