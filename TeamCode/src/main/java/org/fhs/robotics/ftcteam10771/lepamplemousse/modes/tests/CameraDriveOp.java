package org.fhs.robotics.ftcteam10771.lepamplemousse.modes.tests;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.Drive;
import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Alliance;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Components;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.components.Aliases;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.RGB;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.CameraVision;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.Static;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Coordinate;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Rotation;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.entities.Robot;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector.VectorR;

import static org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.RGB.Direction.BOTH;
import static org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.RGB.Direction.LEFT;
import static org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.RGB.Direction.RIGHT;

/**
 * Tests the camera drive class
 * Created by joelv on 2/3/2017.
 */
@Autonomous(name="Camera Drive", group="Test")
public class CameraDriveOp extends LinearOpMode {

    private long lastTime;
    private Config rawSettings;
    private Config.ParsedData settings;
    private Components components;
    private Drive drive;
    private CameraVision cameraVision;
    private RGB rgb;
    private VectorR driveVector = new VectorR(new Coordinate(), new Rotation());
    private boolean backCamera = true;
    float distance_to_stop = 500.0f;
    boolean leftBeacon = false;
    boolean rightBeacon = false;
    Alliance alliance;
    Coordinate coordinate = null;

    Runnable runnable = new Runnable() {
        @Override
        public void run() {
            while (!Thread.currentThread().isInterrupted()) {
                String side = "neither";
                float x = 0.0f;
                float y = 0.0f;
                if (leftBeacon){
                    side = "left";
                }
                else if (rightBeacon){
                    side = "right";
                }
                else side = "neither";
                telemetry.addData("Beacon", side);
                coordinate = cameraVision.updateCoordinates();
                if (coordinate!=null){
                    x = coordinate.getX();
                    y = coordinate.getY();
                }
                telemetry.addData("X", x);
                telemetry.addData("Y", y);
                telemetry.update();
            }
        }
    };

    Thread telemetryThread = new Thread(runnable);

    @Override
    public void runOpMode() throws InterruptedException {

        Config components = new Config(Static.configPath, Static.configCompFileName + Static.configFileSufffix, telemetry, "components");
        if (components.read() == Config.State.DEFAULT_EXISTS) {
            components.create(true);
            if (components.read() == Config.State.DEFAULT_EXISTS)
                components.read(true);
        }

        this.rawSettings = new Config(Static.configPath, Static.configVarFileName + Static.configFileSufffix, telemetry, "settings");
        if (rawSettings.read() == Config.State.DEFAULT_EXISTS) {
            rawSettings.create(true);
            if (rawSettings.read() == Config.State.DEFAULT_EXISTS)
                rawSettings.read(true);
        }

        this.settings = rawSettings.getParsedData();


        this.components = new Components(hardwareMap, telemetry, components);
        this.components.initialize();

        Config.ParsedData drivetrainMotors = settings.subData("drivetrain").subData("motor");

        Aliases.motorMap.get(drivetrainMotors.subData("front_right").getString("map_name"));

        drive = new Drive(driveVector, new Robot(),
                Aliases.motorMap.get(drivetrainMotors.subData("front_right").getString("map_name")),
                Aliases.motorMap.get(drivetrainMotors.subData("front_left").getString("map_name")),
                Aliases.motorMap.get(drivetrainMotors.subData("back_left").getString("map_name")),
                Aliases.motorMap.get(drivetrainMotors.subData("back_right").getString("map_name")),
                settings, telemetry);

        //FIXME: I forgot how the aliases work - Joel
        alliance = (settings.getString("alliance") == "red") ? Alliance.RED_ALLIANCE : Alliance.BLUE_ALLIANCE;
        //rgb = new RGB(hardwareMap.colorSensor.get("color_sensor_left"), hardwareMap.colorSensor.get("color_sensor_right"));
        rgb = new RGB(hardwareMap.colorSensor.get("left_rgb"), hardwareMap.colorSensor.get("right_rgb"),
                hardwareMap.led.get("left_led"), hardwareMap.led.get("right_led"));
        cameraVision = new CameraVision();
        backCamera = cameraVision.usingBackCamera();
        cameraVision.start();
        telemetryThread.start();
        this.lastTime = System.currentTimeMillis();
        rgb.switchLED(BOTH, false);
        waitForStart();

        drive.setRelative(true);
        drive.startVelocity();
        //drive.driveThread.start();
        centerRotate();
        rotate();
        center();
        approach();
        driveVector.setRad(0);
        driveVector.setPolar(0, 0);

        if ((checkBeaconSide(RGB.Direction.LEFT))){
            rgb.indicateCorrcect(LEFT, alliance);
            leftBeacon = true;
            rightBeacon = false;
            press();
            reverse();
        }
        else if (checkBeaconSide(RGB.Direction.RIGHT)){
            rgb.indicateCorrcect(RIGHT, alliance);
            leftBeacon = true;
            rightBeacon = false;
            press();
            reverse();
        }
        else{
            leftBeacon = rightBeacon = false;
        }
        driveVector.setRadius(0);
        driveVector.setPolar(0, 0);
        updatePosition();
        drive.stop();
        telemetryThread.interrupt();
        cameraVision.stop();
    }

    public void center() {
        float marginofError = settings.subData("drive").subData("camera_settings").getFloat("centering_margin");
        if (targeted()) {
            while (Math.abs(cameraVision.getX()) > marginofError) {
                boolean left = backCamera ? (cameraVision.getX() > 0.0) : (cameraVision.getX() < 0.0);
                float theta = left ? (float) Math.PI : 0.0f;
                float radius = settings.subData("drive").subData("camera_settings").getFloat("speed");
                driveVector.setPolar(radius, theta);
            }
        }
        driveVector.setRad(0);
        driveVector.setPolar(0, 0);
    }

    public void approach() {
        distance_to_stop = settings.subData("drive").subData("camera_settings").getFloat("distance_to_stop");
        if (targeted()) {
            while (targeted() && Math.abs(cameraVision.getZ()) > distance_to_stop) {
                float theta = backCamera ? 3.0f * (float) Math.PI / 2.0f : (float) Math.PI / 2.0f;
                float radius = settings.subData("drive").subData("camera_settings").getFloat("speed");
                //todo put speed
                driveVector.setPolar(radius, theta);
            }
        }
        driveVector.setRad(0);
        driveVector.setPolar(0, 0);
    }

    public void rotate(){
        float rotate_margin = settings.subData("drive").subData("camera_settings").getFloat("angle_margin");
        float rotate_speed = settings.subData("drive").subData("camera_settings").getFloat("rotate_speed");
        if (targeted()){
            while(Math.toDegrees(Math.abs(cameraVision.getAngleToTurn()))>rotate_margin){
                driveVector.setRad((float)Math.copySign(rotate_speed, cameraVision.getAngleToTurn()));
            }
        }
        driveVector.setRad(0);
        driveVector.setPolar(0, 0);
    }

    //todo some more testing on that
    public void centerRotate(){
        float rotate_margin = (float) Math.toRadians(settings.subData("drive").subData("camera_settings").getFloat("angle_margin"));
        float rotate_speed = settings.subData("drive").subData("camera_settings").getFloat("rotate_speed");
        float marginofError = settings.subData("drive").subData("camera_settings").getFloat("centering_margin");
        float radius = settings.subData("drive").subData("camera_settings").getFloat("speed");
        float rotate_factor = settings.subData("drive").subData("camera_settings").getFloat("rotate_factor");
        while (targeted() && Math.abs(cameraVision.getX()) > marginofError && Math.abs(cameraVision.getAngleToTurn())>rotate_margin) {
            boolean left = backCamera ? (cameraVision.getX() > 0.0) : (cameraVision.getX() < 0.0);
            float rotate = (float)(radius/(rotate_factor*rotate_speed) * cameraVision.getAngleToTurn());
            float theta = left ? (float) Math.PI : 0.0f;
            driveVector.setPolar(radius, theta);
            driveVector.setRad((float)Math.copySign(rotate, cameraVision.getAngleToTurn()));
        }
        driveVector.setRad(0);
        driveVector.setPolar(0, 0);
    }

    public boolean checkBeaconSide(RGB.Direction direction){
        if (targeted()){
            boolean reached = false;
            float distance = settings.subData("drive").subData("camera_settings").getFloat("beacon_check_distance");
            float theta = 0.0f;
            switch (direction){
                case LEFT:
                    theta = backCamera ? (float)Math.PI : 0.0f;
                    break;
                case RIGHT:
                    theta = backCamera ? 0.0f : (float)Math.PI;
                    break;
                default:
                    theta = 0.0f;
            }
            reached = (direction==LEFT) ? cameraVision.getX() > distance :
                    cameraVision.getX() < distance;
            while (targeted() && reached){
                float radius = distance - (float)Math.abs(cameraVision.getX());
                //todo put power cutback ratio in settings config
                radius *= settings.subData("drive").subData("camera_settings").getFloat("speed");
                //todo put speed under camera_settings under drive
                driveVector.setTheta(theta);
                driveVector.setRadius(radius);
                reached = (direction==LEFT) ? cameraVision.getX() > distance :
                        cameraVision.getX() < distance;
            }
            return rgb.isSide(alliance, direction);
        }
        else return false;
    }

    private void press(){
        float theta = backCamera ? 3.0f * (float)Math.PI / 2.0f : 0.0f;
        float radius = settings.subData("drive").subData("camera_settings").getFloat("speed");
        radius /= 4f;
        long waitTime = settings.subData("drive").subData("camera_settings").getInt("wait_for_press");
        long lastTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - lastTime < waitTime){
            driveVector.setPolar(radius, theta);
        }
        driveVector.setPolar(0f, 0f);
    }

    private void reverse(){
        float theta = backCamera ? 0.0f : 3.0f * (float)Math.PI / 2.0f;
        float radius = settings.subData("drive").subData("camera_settings").getFloat("speed");
        radius /= 4f;
        long waitTime = settings.subData("drive").subData("camera_settings").getInt("wait_for_press");
        waitTime /= 2f;
        long lastTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - lastTime < waitTime){
            driveVector.setPolar(radius, theta);
        }
    }

    private void updatePosition(){
        drive.initPosition(cameraVision.updateCoordinates().getX(),
                cameraVision.updateCoordinates().getY());
    }

    private boolean targeted(){
        return cameraVision.imageInSight(cameraVision.target());
    }
}
