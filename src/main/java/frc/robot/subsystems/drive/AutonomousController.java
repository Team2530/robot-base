package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.util.constants.ChoreoConstants;

public class AutonomousController
        extends PPHolonomicDriveController
{
    private static StructPublisher<Pose2d> autoPosePublisher =
        NetworkTableInstance
        .getDefault()
        .getStructTopic("Auto target pose", Pose2d.struct).publish();

    public AutonomousController() {
        super(
            new PIDConstants(
                ChoreoConstants.PID.Translation.P,
                ChoreoConstants.PID.Translation.I,
                ChoreoConstants.PID.Translation.D
            ),
            new PIDConstants(
                ChoreoConstants.PID.Heading.P,
                ChoreoConstants.PID.Heading.I,
                ChoreoConstants.PID.Heading.D
            )
        );
    }

    @Override
    public ChassisSpeeds calculateRobotRelativeSpeeds(
        Pose2d pose,
        PathPlannerTrajectoryState target
    ) {
        ChassisSpeeds speeds = super.calculateRobotRelativeSpeeds(pose,target);

        SmartDashboard.putNumber(
            "Auto/x",
            target.pose.getX()
        );
        SmartDashboard.putNumber(
            "Auto/y",
            target.pose.getY()
        );
        SmartDashboard.putNumber(
            "Auto/vx",
            target.fieldSpeeds.vxMetersPerSecond
        );
        SmartDashboard.putNumber(
            "Auto/vy",
            target.fieldSpeeds.vyMetersPerSecond
        );
        SmartDashboard.putNumber(
            "Auto/heading",
            target.heading.getRotations()
        );
        SmartDashboard.putNumber(
            "Auto/omega",
            target.fieldSpeeds.omegaRadiansPerSecond
        );
        autoPosePublisher.set(
            target.pose
        );

        return speeds;
    }
}
