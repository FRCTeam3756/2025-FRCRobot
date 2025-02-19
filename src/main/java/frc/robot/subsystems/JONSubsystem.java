// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ai.JONCommunication;

public class JONSubsystem extends SubsystemBase {
    private JONCommunication jon = new JONCommunication();

    public JONSubsystem() {}

    @Override
    public void periodic() {
        jon.getAIRequest();
    }
}
