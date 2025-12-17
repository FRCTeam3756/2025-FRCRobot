// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controller {
    public static final int CONTROLLER_PORT = 0;
    public static final double DEADZONE = 0.1;

    public static final CommandXboxController controller = new CommandXboxController(Controller.CONTROLLER_PORT);

    // Joysticks
    public static final Trigger driveTurboButton = controller.leftStick();
    public static final Trigger blank = controller.rightStick();

    // D-Pad
    public static final Trigger elevatorDownButton = controller.povDown();
    public static final Trigger elevatorUpButton = controller.povUp();
    public static final Trigger blank1 = controller.povLeft();
    public static final Trigger blank2 = controller.povRight();

    // Face Buttons
    public static final Trigger driveSlugButton = controller.y();
    public static final Trigger clawOuttakeButton = controller.x();
    public static final Trigger driveSlowButton = controller.b();
    public static final Trigger clawIntakeButton = controller.a();

    // Back Buttons
    public static final Trigger clawTiltUp = controller.rightBumper();
    public static final Trigger clawTiltDown = controller.leftBumper();
    public static final Trigger blank3 = controller.leftTrigger();
    public static final Trigger blank4 = controller.rightTrigger();

    // Top Buttons
    public static final Trigger climbButton = controller.start();
    public static final Trigger resetGyroScope = controller.back();
}
