// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.constants.CANConstants;

public class PowerSubsystem {

    public final PowerDistribution powerDistributionHub;

    public PowerSubsystem() {
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
