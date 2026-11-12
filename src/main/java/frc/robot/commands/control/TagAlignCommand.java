package frc.robot.commands.control;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.limelight.LimelightHelpers.setLimelightNTDouble;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.util.BlipControllerCommand;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.util.constants.DriveConstants.AlignmentPid;
import frc.robot.util.constants.DriveConstants.AlignemntTolerance;
import frc.robot.util.limelight.LimelightHelpers;
import frc.robot.util.limelight.LimelightHelpers.RawFiducial;
import frc.robot.util.constants.LimelightConstants;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.commands.util.BlipControllerCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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
    public final Angle TARGET_ANGLE;
    public final int TAG_ID;
    public final String LIMELIGHT_ID;
    public final XboxController CONTROLLER;
    
    Boolean SeeingTag = false;

    public TagAlignCommand(SwerveSubsystem subsystem, Distance targetForward, Distance targetStrafe, int tagID, String LimelightID, XboxController Controller) {
        this.subsystem = subsystem;
        this.TAG_ID = tagID;
        m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        addRequirements(subsystem);


        m_xController.setSetpoint(AlignemntTolerance.FORWARD.in(Meters)); 
        m_yController.setSetpoint(AlignemntTolerance.STRAFE.in(Meters)); 
        m_yawController.setSetpoint(AlignemntTolerance.ROTATION.in(Degree));  

        TARGET_FORWARD = targetForward;
        TARGET_STRAFE = targetStrafe;
        TARGET_ANGLE = Degree.of(0); // Assuming we want to face the target directly

        LIMELIGHT_ID = LimelightID;

        CONTROLLER = Controller;
    }

    @Override
    public void initialize() {
        // Reset PID controllers to ensure they start fresh
        m_xController.reset();
        m_yController.reset();
        m_yawController.reset();


        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(LIMELIGHT_ID);
        for (RawFiducial fiducial : fiducials) {
            if (fiducial.id == TAG_ID) {
                SeeingTag = true;
                LimelightHelpers.setPriorityTagID(LIMELIGHT_ID, TAG_ID);
            } else {
                new BlipControllerCommand(CONTROLLER, 3);
            }
        }

        SmartDashboard.putBoolean(
            "ApriltagAlignment/FoundTag",
            SeeingTag
        );


    }

    @Override
    public void execute() {
        
        if (LimelightHelpers.getFiducialID(LIMELIGHT_ID) == TAG_ID) {

            SeeingTag = true;

            double[] targetPose = LimelightHelpers.getTargetPose_RobotSpace(LIMELIGHT_ID);

            if (targetPose.length < 6) { // Check that values actually got received. 
                double currentStrafe = targetPose[0];   // Left/Right offset (meters)
                double currentDistance = targetPose[1]; // Forward distance (meters)
                double currentYaw = targetPose[5];      // Yaw Angle (degrees)

                subsystem.drive(
                    new Translation2d(
                        m_xController.calculate(currentDistance, TARGET_FORWARD.in(Meters)),
                        m_yController.calculate(currentStrafe, TARGET_STRAFE.in(Meters))
                    ), 
                    m_yawController.calculate(currentYaw, TARGET_ANGLE.in(Degrees))
                );
            }

        } else {
            SeeingTag = false;

            subsystem.drive(new Translation2d(0,0), 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop Driving when Alligned
        subsystem.drive(
            new Translation2d(0,0),
            0
        );

        new BlipControllerCommand(CONTROLLER, 1);

    }

    @Override
    public boolean isFinished() {
        // TODO: Make sure setpoint isnt being reached during initialization. 
        return !SeeingTag || (m_xController.atSetpoint() && m_yController.atSetpoint() && m_yawController.atSetpoint());
    }
}
    