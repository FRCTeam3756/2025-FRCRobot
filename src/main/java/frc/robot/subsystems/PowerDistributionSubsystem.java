// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.connection.PortConstants;

public class PowerDistributionSubsystem extends SubsystemBase {

    public final PowerDistribution powerDistributionSubsystem;

    public PowerDistributionSubsystem() {
        powerDistributionSubsystem = new PowerDistribution(PortConstants.REV_PDH_ID, PowerDistribution.ModuleType.kRev);
    }

    @Override
    public void periodic() {
    }

    public void setSwitchableChannel(boolean enable) {
        powerDistributionSubsystem.setSwitchableChannel(enable);
    }

    public void clearStickyFaults() {
        powerDistributionSubsystem.clearStickyFaults();
    }

    public double getChannelCurrent(int channel) {
        return powerDistributionSubsystem.getCurrent(channel);
    }
}
