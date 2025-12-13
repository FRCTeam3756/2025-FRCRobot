package frc.robot.hardware;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.constants.CANConstants;

public class REVPowerDistributionHub {

    public final PowerDistribution powerDistributionHub;

    public REVPowerDistributionHub() {
        powerDistributionHub = new PowerDistribution(CANConstants.REV_PDH_ID, PowerDistribution.ModuleType.kRev);
    }

    public void setSwitchableChannel(boolean enable) {
        powerDistributionHub.setSwitchableChannel(enable);
    }

    public void clearStickyFaults() {
        powerDistributionHub.clearStickyFaults();
    }

    public void getChannelCurrent(int channel) {
        powerDistributionHub.getCurrent(channel);
    }
}
