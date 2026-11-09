package frc.robot.commands.control;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.util.constants.DriveConstants.AlignmentPid;

public class TagAlignCommand extends Command {
    private final SwerveSubsystem subsystem;
    private final NetworkTable m_limelightTable;

    private final PIDController m_xController =  
        new PIDController(
            AlignmentPid.Forward.P, 
            AlignmentPid.Forward.I, 
            AlignmentPid.Forward.D
        ); // Forward/Backward (Distance)
    private final PIDController m_yController = 
        new PIDController(
            AlignmentPid.Strafe.P, 
            AlignmentPid.Strafe.I, 
            AlignmentPid.Strafe.D
        ); // Left/Right (Strafe)
    private final PIDController m_yawController = 
        new PIDController(
            AlignmentPid.Rotation.P, 
            AlignmentPid.Rotation.I,
            AlignmentPid.Rotation.D
        ); // Rotation

    
    public final Distance TARGET_FORWARD;
    public final Distance TARGET_STRAFE;
    
    public TagAlignCommand(SwerveSubsystem subsystem, Distance targetForward, Distance targetStrafe) {
        this.subsystem = subsystem;
        m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        addRequirements(subsystem);

        // Set tolerances to avoid jitter near the target
        m_xController.setTolerance(0.03); // 3 cm
        m_yController.setTolerance(0.03); 
        m_yawController.setTolerance(1.5);  // 1.5 degrees

        TARGET_FORWARD = targetForward;
        TARGET_STRAFE = targetStrafe;
    }

    @Override
    public void initialize() {
        // Reset PID controllers to ensure they start fresh
        m_xController.reset();
        m_yController.reset();
        m_yawController.reset();
    }

    @Override
    public void execute() {
        // TODO: Alignment Logic
    }

    @Override
    public void end(boolean interrupted) {
        // Stop Driving when Alligned
        subsystem.drive(
            new Translation2d(0,0),
            0
        );
    }

    @Override
    public boolean isFinished() {
        // TODO: Check if the robot is aligned with the target
        return false;
    }
}
