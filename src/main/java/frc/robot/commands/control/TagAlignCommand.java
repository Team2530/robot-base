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
    
    public TagAlignCommand(SwerveSubsystem subsystem, Distance targetForward, Distance targetStrafe, int tagID) {
        this.subsystem = subsystem;
        this.TAG_ID = tagID;
        m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        addRequirements(subsystem);

        // Set tolerances to avoid jitter near the target
        m_xController.setTolerance(AlignemntTolerance.FORWARD.in(Meters)); 
        m_yController.setTolerance(AlignemntTolerance.STRAFE.in(Meters)); 
        m_yawController.setTolerance(AlignemntTolerance.ROTATION.in(Degree));  

        TARGET_FORWARD = targetForward;
        TARGET_STRAFE = targetStrafe;
        TARGET_ANGLE = Degree.of(0); // Assuming we want to face the target directly
    }

    @Override
    public void initialize() {
        // Reset PID controllers to ensure they start fresh
        m_xController.reset();
        m_yController.reset();
        m_yawController.reset();

        // Find the closest Limelight to the target and set its ID filters
        // TODO: Improve This (This may not even be neccesary if we decide only a specifc limelight should be used)
        /*
                        ⠀⣠⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣰⣄⡀⠀⠀⠀
                ⣿⠋⣻⢲⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⡀⠀⠙⣆⠀⠀
                ⣿⣦⡀⠀⢘⢷⣀⠀⢀⣰⢟⠛⣦⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⠀⠀⢀⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡀⡄⠀⠀⠈⢷⡀⠀⢹⢷⠀
                ⠈⢿⡄⠀⠈⠉⠈⠹⣟⠋⠀⣵⠟⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣿⣤⣤⣼⣷⣄⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢰⡏⠙⣦⡀⠀⣼⡇⠀⠀⢸⡇
                ⠀⠈⣇⠀⠀⠀⠀⠞⠉⢀⡾⠁⠀⠀⠀⠀⠀⠀⠀⣠⡶⣟⢟⣉⠀⠀⣀⣀⣉⠉⠉⠓⠦⣄⠀⠀⠀⠀⠀⠸⣇⠀⢘⣧⣴⠿⠀⠀⠀⢸⡇
                ⠀⠀⠛⣇⠀⢈⠀⠀⠀⠘⡇⠀⠀⠀⠀⠀⢀⣤⢾⣷⡶⠛⠛⠛⠀⠀⠈⠉⠉⠙⠳⡄⠀⠈⠳⣄⠀⠀⠀⠀⢸⡇⠀⠙⠁⠀⠀⠀⠀⢺⡇
                ⠀⠀⠀⢿⡀⠀⠀⠒⠀⣈⡇⠀⠀⠀⠀⢠⡟⠑⠁⠀⢀⡀⣠⣴⣷⣶⣶⣶⣤⣀⠀⠀⠀⠀⠀⠈⢳⡄⠀⠀⢸⡇⠀⠀⠀⠀⠀⢠⢾⠟⠁
                ⠀⠀⠀⠈⠳⣄⠀⠀⢨⣿⠉⠀⠀⠀⢠⡟⠀⠀⠀⠀⣤⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣄⠀⠀⠀⠀⢈⢻⡄⠈⠉⠳⣇⠀⢀⢄⣤⠞⠁⠀⠀
                ⠀⠀⠀⠀⠀⠈⠹⠏⠏⠁⠀⠀⠀⠀⡿⠀⠀⠀⢀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡆⠀⠀⠀⠈⢏⡇⠀⠀⠀⠈⠹⠉⠉⠀⠀⠀⠀⠀
                ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡇⠀⠀⠀⣰⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡆⠀⠀⠀⠈⣻⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
                ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣇⠀⠀⢰⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⡄⠀⠀⠀⣿⠂⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
                ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣯⠀⠂⣿⣿⣿⣿⣿⡿⠿⠉⠉⠉⠛⠿⣿⣿⣿⣿⣿⣧⠀⠀⢰⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
                ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢻⠀⠠⣿⣿⣿⣿⠯⠀⠀⠀⠀⠀⠀⠀⠛⢿⣿⣿⣿⣿⠏⠀⣾⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
                ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⣇⠺⣿⣿⡿⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢹⢿⣿⣿⡃⢼⡏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
                ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢹⣤⠛⠏⠀⠀⠀⠀⠀⡀⠀⠀⠀⠀⠀⠀⠀⠈⣛⣿⣳⣿⠇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
                ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠉⠛⠓⠶⣄⡀⠀⠀⠀⠀⠀⠀⠀⢀⡀⣶⣶⠿⠟⠻⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
                ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠙⠛⠒⠶⠶⠒⠲⠶⠞⠛⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
         */

        List<String> activeLimelights = new java.util.ArrayList<>();
        List<Distance> activeLimelightDistance = new java.util.ArrayList<>();

        String closestLimelightID = "";
        double closestDistance = Double.MAX_VALUE;

        Boolean foundTag;

        for (Limelight limelight : LimelightConstants.limelightConfigs) {

            String ID = limelight.getId(); 
            RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(ID);

            for (RawFiducial fiducial : fiducials) {
                if (fiducial.id == TAG_ID) {
                    activeLimelights.add(ID);
                    activeLimelightDistance.add(
                        Meters.of(
                            fiducial.distToCamera
                        )
                    );
                }
            }
        }

        if (activeLimelights.size() > 0) {
            int closestIndex = 0;
            for (int i = 1; i < activeLimelightDistance.size(); i++) {
                if (activeLimelightDistance.get(i).in(Meters) < activeLimelightDistance.get(closestIndex).in(Meters)) {
                    closestIndex = i;
                }
            }

            closestLimelightID = activeLimelights.get(closestIndex);

            LimelightHelpers.SetFiducialIDFiltersOverride(
                closestLimelightID,
                new int[]{TAG_ID}
            );

            foundTag = true;
        }else{
            foundTag = false;
        }

        SmartDashboard.putBoolean(
            "ApriltagAlignment/FoundTag", 
            foundTag
        );
        SmartDashboard.putString(
            "ApriltagAlignment/LimelightID", 
            closestLimelightID
        );
        SmartDashboard.putNumber(
            "ApriltagAlignment/LimelightDistance", 
            closestDistance
        );
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
