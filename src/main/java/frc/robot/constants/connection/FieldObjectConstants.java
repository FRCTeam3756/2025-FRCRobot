package frc.robot.constants.connection;

public class FieldObjectConstants {
    public static class FieldObject {
        public String camera;
        public double x;
        public double y;
        public double timestamp;

        public FieldObject(String camera, double x, double y, double timestamp) {
            this.camera = camera;
            this.x = x;
            this.y = y;
            this.timestamp = timestamp;
        }
    }

    public static class OpponentRobot extends FieldObject {
        public OpponentRobot(String camera, double x, double y, double timestamp) {
            super(camera, x, y, timestamp);
        }
    }

    public static class TeammateRobot extends FieldObject {
        public TeammateRobot(String camera, double x, double y, double timestamp) {
            super(camera, x, y, timestamp);
        }
    }

    public static class Algae extends FieldObject {
        public Algae(String camera, double x, double y, double timestamp) {
            super(camera, x, y, timestamp);
        }
    }

    public static class Coral extends FieldObject {
        public Coral(String camera, double x, double y, double timestamp) {
            super(camera, x, y, timestamp);
        }
    }
}
