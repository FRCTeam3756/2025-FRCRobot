// package frc.robot.subsystems;

// import java.util.Map;

// import edu.wpi.first.cscore.CvSink;
// import edu.wpi.first.cscore.CvSource;
// import edu.wpi.first.cscore.MjpegServer;
// import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.util.PixelFormat;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class CameraSubsystem extends SubsystemBase {
//     private final UsbCamera camera;
//     private final MjpegServer mjpegServer1, mjpegServer2;
//     private final CvSink cvSink;
//     private final CvSource outputStream;

//     public CameraSubsystem() {
//         camera = new UsbCamera("Front Camera", 0);

//         mjpegServer1 = new MjpegServer("serve_Front Camera", 1181);
//         mjpegServer1.setSource(camera);

//         cvSink = new CvSink("opencv_Front Camera");
//         cvSink.setSource(camera);

//         outputStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30);
//         mjpegServer2 = new MjpegServer("Front Camera", 1182);
//         mjpegServer2.setSource(outputStream);
//     }

//     @Override
//     public void periodic() {}
    
//     public void setupShuffleboardCameraFeed() {
//         Shuffleboard.getTab("Driver View")
//             .addCamera("Front Camera", "test", "mjpg:http://10.37.56.2:1811/?action=stream")
//             .withProperties(Map.of("showControls", false));
//     }
// }
