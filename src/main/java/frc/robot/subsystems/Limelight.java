package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.LimelightHelpers;

/**
 * The Limelight Subsystem handles interactions with one Limelight.
 *
 * This subsystem manages the pose to output to Shuffleboard.
 */
public class Limelight extends SubsystemBase {
    public enum LimelightType {
        // LL1 doesn't have specs listed, so these could be incorrect
        LL1(54, 41), LL2(62.5, 48.9), LL2Plus(62.5, 48.9),
        LL3(62.5, 48.9),
        LL3G(82, 56.2),
        LL4(82, 56.2);

        private final double HFOV;
        private final double VFOV;

        private LimelightType(double HFOV, double VFOV) {
            this.HFOV = HFOV;
            this.VFOV = VFOV;
        }
    }

    private final LimelightType limelightType;
    private final String id;
    private double lastFrame = 0;
    private StructPublisher<Pose2d> posePublisher;

    public Limelight(
        LimelightType limelightType,
        String id,
        Pose3d pose
    ) {
        this.limelightType = limelightType;
        this.id = id;

        posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic(
                "SmartDashboard/Odometry/" + getID() + "/pose",
                Pose2d.struct
            )
            .publish();

        LimelightHelpers.setCameraPose_RobotSpace(
            id,
            pose.getX(),
            pose.getY(),
            pose.getZ(),
            Units.radiansToDegrees(
                pose.getRotation().getX()
            ),
            Units.radiansToDegrees(
                pose.getRotation().getY()
            ),
            Units.radiansToDegrees(
                pose.getRotation().getZ()
            )
        );
    
        LimelightHelpers.SetIMUAssistAlpha(
            id,
            0.03
        );

        LimelightHelpers.SetIMUMode(
            id,
            1
        );
    }

    @Override
    public void periodic() {
        if (
            RobotContainer.swerveDriveSubsystem.getAngularVelocity()
                .abs(DegreesPerSecond)
            > 2
        ) {
            RobotContainer.LLContainer.setIMUMode(4);
        } else {
            RobotContainer.LLContainer.setIMUMode(3);
        }
    }

    public void publish(Pose2d pose) {
        posePublisher.set(pose);
    }

    public int numTargets() {
        return LimelightHelpers.getRawFiducials(id).length;
    }


    public void setIMUMode(int mode) {LimelightHelpers.SetIMUMode(id, mode);}

    public double getLastFrameTime(){return lastFrame;}
    public void setLastFrame(double lastFrameTime){lastFrame = lastFrameTime;}

    public double getVFOV() {return limelightType.VFOV;}
    public double getHFOV() {return limelightType.HFOV;}

    @Override
    public String toString() {
        return this.id;
    }

    public String getID() {
        return this.id;
    }
}
