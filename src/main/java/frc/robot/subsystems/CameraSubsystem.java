package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
    private final UsbCamera camera;
    private final VideoSink videoSink;

    public CameraSubsystem() {
        camera = new UsbCamera("Microsoft Lifecam", 0);
        // camera.setResolution(640, 480);

        videoSink = CameraServer.getVideo();
        videoSink.setSource(camera);
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
