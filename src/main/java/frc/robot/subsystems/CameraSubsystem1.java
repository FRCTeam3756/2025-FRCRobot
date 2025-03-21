package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem1 extends SubsystemBase {
    private final UsbCamera camera;

    public CameraSubsystem1() {
        camera = new UsbCamera("Front Camera", 0);
        camera.setFPS(30);
        camera.setResolution(320, 240);

        CameraServer.startAutomaticCapture(camera);
    }

    @Override
    public void periodic() {}

    private UsbCamera getCamera() {
        return camera;
    }

    
    public void setupShuffleboardCameraFeed() {
        ShuffleboardTab tab = Shuffleboard.getTab("Driver View");
        
        tab.add("Camera Feed", getCamera())
            .withPosition(0, 0)
            .withSize(3, 3);
    }
}
