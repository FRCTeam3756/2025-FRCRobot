package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.UltrasonicConstants;

public class UltrasonicSubsystem extends SubsystemBase {
    private final DigitalOutput trigger;
    private final DigitalInput echo;

    public UltrasonicSubsystem() {
        trigger = new DigitalOutput(UltrasonicConstants.TRIGGER_PORT);
        echo = new DigitalInput(UltrasonicConstants.ECHO_PORT);
    }

    public double getDistanceCM() {
        trigger.set(false);
        Timer.delay(0.000002);
        trigger.set(true);
        Timer.delay(0.00001);
        trigger.set(false);

        Timer timer = new Timer();
        timer.start();
        while (!echo.get()) {
            if (timer.get() > UltrasonicConstants.MAX_SENSOR_TIME) return -1;
        }
        double startTime = Timer.getFPGATimestamp();

        while (echo.get()) {
            if (timer.get() > UltrasonicConstants.MAX_SENSOR_TIME) return -1;
        }
        double endTime = Timer.getFPGATimestamp();

        double timeElapsed = endTime - startTime;

        return (timeElapsed * UltrasonicConstants.SPEED_OF_SOUND_IN_CM_PER_SEC) / 2.0;
    }

    @Override
    public void periodic() {}
}
