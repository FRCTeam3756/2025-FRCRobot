// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;

    private static final CommandXboxController controller = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);

    public static final Trigger elevatorUpButton = controller.povRight();
    public static final Trigger elevatorDownButton = controller.povLeft();
    public static final Trigger climbUpButton = controller.povUp();
    public static final Trigger climbDownButton = controller.povDown();
    public static final Trigger intakeButton = controller.rightBumper();
    public static final Trigger shootProcessorButton = controller.leftBumper();
    public static final Trigger shootCoralButton = controller.rightTrigger();
    public static final Trigger shootBargeButton = controller.leftTrigger();
    public static final Trigger autoIntakeButton = controller.a();
    public static final Trigger autoSpeakerButton = controller.b();
    public static final Trigger autoAmpButton = controller.x();
    public static final Trigger resetGyroButton = controller.y();
    public static final Trigger driveTurboButton = controller.leftStick();

    public static final double DEADZONE = 0.05;

}
