package frc.robot.subsystems.util;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.RobotConstants;

public class PDHSubsystem extends SubsystemBase {

    private static final PowerDistribution PDH = new PowerDistribution(
        RobotConstants.CANIDs.PDH,
        ModuleType.kRev
    );

    @Override
    public void periodic() {
        // wattage and joule draw is not supported on the rev pdh
        SmartDashboard.putNumber(
            "PDH/totalCurrentDraw",
            PDH.getTotalCurrent()
        );

        // [documentation](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/PowerDistribution.html#getAllCurrents())
        // only specifies that this works for the PDP, does not mention rev PDH
        // but i don't see why not
        SmartDashboard.putNumberArray(
            "PDH/channelCurrents",
            PDH.getAllCurrents()
        );

        SmartDashboard.putNumber(
            "PDH/batteryVoltage",
            PDH.getVoltage()
        );

        SmartDashboard.putBoolean(
            "PDH/switchableChannelEnabled",
            getSwitchableChannel()
        );
    }

    public boolean getSwitchableChannel() {
        return PDH.getSwitchableChannel();
    }

    public void setSwitchableChannel(boolean value) {
        PDH.setSwitchableChannel(value);
    }
}
