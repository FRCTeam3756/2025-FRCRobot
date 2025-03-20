// Copyright (c) FRC Team 3756 RamFerno.
// Open Source Software; you can modify and/or share it under the terms of
// the license viewable in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Controller {
    public static final int CONTROLLER_PORT = 0;
    public static final double DEADZONE = 0.05;

    public static final CommandXboxController controller = new CommandXboxController(Controller.CONTROLLER_PORT);

    // Joysticks
    public static final Trigger driveTurboButton = controller.leftStick();
    public static final Trigger blank5 = controller.rightStick();

    // D-Pad
    public static final Trigger elevatorUpButton = controller.povUp();
    public static final Trigger elevatorDownButton = controller.povDown();
    public static final Trigger blank = controller.povLeft();
    public static final Trigger blank1 = controller.povRight();

    // Face Buttons
    public static final Trigger blank2 = controller.y();
    public static final Trigger blank3 = controller.x();
    public static final Trigger driveSlowButton = controller.b();
    public static final Trigger blank4 = controller.a();

    // Back Buttons
    public static final Trigger clawTiltUp = controller.rightBumper();
    public static final Trigger clawTiltDown = controller.leftBumper();
    public static final Trigger clawOuttakeButton = controller.leftTrigger();
    public static final Trigger clawIntakeButton = controller.rightTrigger();

    // Top Buttons
    public static final Trigger climbButton = controller.start();
    public static final Trigger resetGyroButton = controller.back();

    // Button Box Buttons
    private static final GenericHID buttonBox = new GenericHID(0);

    public static final Trigger button1 = new Trigger(() -> buttonBox.getRawButton(1));
    public static final Trigger elevatorL1Button = new Trigger(() -> buttonBox.getRawButton(2));
    public static final Trigger button3 = new Trigger(() -> buttonBox.getRawButton(3));
    public static final Trigger elevatorL2Button = new Trigger(() -> buttonBox.getRawButton(4));
    public static final Trigger button5 = new Trigger(() -> buttonBox.getRawButton(5));
    public static final Trigger elevatorL3Button = new Trigger(() -> buttonBox.getRawButton(6));
    public static final Trigger button7 = new Trigger(() -> buttonBox.getRawButton(7));
    public static final Trigger elevatorL4Button = new Trigger(() -> buttonBox.getRawButton(8));
}
