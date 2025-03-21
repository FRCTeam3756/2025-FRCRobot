package frc.robot.subsystems;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem2 extends SubsystemBase {
    private final UsbCamera camera;
    private final MjpegServer mjpegServer1, mjpegServer2;
    private final CvSink cvSink;
    private final CvSource outputStream;

    public CameraSubsystem2() {
        camera = new UsbCamera("Front Camera", 0);
        mjpegServer1 = new MjpegServer("Front Camera", 1181);
        mjpegServer1.setSource(camera);
        cvSink = new CvSink("Front Camera");
        cvSink.setSource(camera);

        outputStream = new CvSource("Front Camera", PixelFormat.kMJPEG, 640, 480, 30);
        mjpegServer2 = new MjpegServer("Front Camera", 1182);
        mjpegServer2.setSource(outputStream);

        setupShuffleboardCameraFeed();
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
