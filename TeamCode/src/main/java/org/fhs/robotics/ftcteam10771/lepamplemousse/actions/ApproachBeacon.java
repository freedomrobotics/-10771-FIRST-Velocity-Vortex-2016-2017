package org.fhs.robotics.ftcteam10771.lepamplemousse.actions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Alliance;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.RGB;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.CameraVision;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Coordinate;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector.VectorR;

import static org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.RGB.Direction.BOTH;
import static org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.RGB.Direction.LEFT;

/**
 * Created by Adam Li on 2/17/2017.
 * Derived from the CameraDriveOp class
 * todo cleanup
 */

public class ApproachBeacon {
    private final Config.ParsedData settings;
    private final Drive drive;
    private final Alliance alliance;
    private final RGB rgb;
    private final CameraVision cameraVision;
    private long lastTime;
    private boolean backCamera;
    private VectorR driveVector;
    private final LinearOpMode linearOpMode;
    private final HardwareMap hardwareMap;
    private boolean rightSideChecked = false;

    public ApproachBeacon(Config.ParsedData settings, Drive drive, LinearOpMode linearOpMode){
        this.drive = drive;
        this.settings = settings;
        this.driveVector = drive.getVectorR();
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;

        alliance = (settings.getString("alliance") == "red") ? Alliance.RED_ALLIANCE : Alliance.BLUE_ALLIANCE;
        //rgb = new RGB(hardwareMap.colorSensor.get("color_sensor_left"), hardwareMap.colorSensor.get("color_sensor_right"));
        rgb = new RGB(hardwareMap.colorSensor.get("left_rgb"),
                hardwareMap.led.get("left_led"));
        cameraVision = new CameraVision(false);
        backCamera = cameraVision.usingBackCamera();
        //cameraVision.start();
        this.lastTime = System.nanoTime();
        rgb.switchLED(false);
    }

    public void center() {
        drive.setRelative(true);
        drive.startVelocity();
        float marginofError = settings.subData("drive").subData("camera_settings").getFloat("centering_margin");
        if (targeted()) {
            while (Math.abs(cameraVision.getX()) > marginofError && linearOpMode.opModeIsActive() && targeted()) {
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
        drive.setRelative(true);
        drive.startVelocity();
        double distance_to_stop = settings.subData("drive").subData("camera_settings").getFloat("distance_to_stop");
        if (targeted()) {
            while (targeted() && Math.abs(cameraVision.getZ()) > distance_to_stop && linearOpMode.opModeIsActive() && targeted()) {
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
        drive.setRelative(true);
        drive.startVelocity();
        float rotate_margin = settings.subData("drive").subData("camera_settings").getFloat("angle_margin");
        float rotate_speed = settings.subData("drive").subData("camera_settings").getFloat("rotate_speed");
        if (targeted()){
            while(Math.toDegrees(Math.abs(cameraVision.getAngleToTurn()))>rotate_margin && targeted() && linearOpMode.opModeIsActive()){
                driveVector.setRad((float)Math.copySign(rotate_speed, cameraVision.getAngleToTurn()));
            }
        }
        driveVector.setRad(0);
        driveVector.setPolar(0, 0);
    }

    //todo some more testing on that
    public void centerRotate(){
        drive.setRelative(true);
        drive.startVelocity();
        float rotate_margin = (float) Math.toRadians(settings.subData("drive").subData("camera_settings").getFloat("angle_margin"));
        float rotate_speed = settings.subData("drive").subData("camera_settings").getFloat("rotate_speed");
        float marginofError = settings.subData("drive").subData("camera_settings").getFloat("centering_margin");
        float radius = settings.subData("drive").subData("camera_settings").getFloat("speed");
        float rotate_factor = settings.subData("drive").subData("camera_settings").getFloat("rotate_factor");
        while (targeted() && (Math.abs(cameraVision.getX()) > marginofError || Math.abs(cameraVision.getAngleToTurn())>rotate_margin) && linearOpMode.opModeIsActive()) {
            boolean left = backCamera ? (cameraVision.getX() > 0.0) : (cameraVision.getX() < 0.0);
            float rotate = (float)(radius/(rotate_factor*rotate_speed) * cameraVision.getAngleToTurn());
            float theta = left ? (float) Math.PI : 0.0f;
            driveVector.setPolar(radius, theta);
            driveVector.setRad((float)Math.copySign(rotate, cameraVision.getAngleToTurn()));
        }
        driveVector.setRad(0);
        driveVector.setPolar(0, 0);
    }

    public CameraVision cameraVision(){
        return  cameraVision;
    }

    private boolean targeted(){
        return cameraVision.imageInSight(cameraVision.target());
    }

    public Alliance checkRightSide(){
        float theta = 0f;
        float radius = 0.3f; // todo put speed in config
        drive.startVelocity();
        //while (the color sensor does not see a bright color)
        while (linearOpMode.opModeIsActive()){
            driveVector.setPolar(radius, theta);
        }
        driveVector.setPolar(0f, 0f);
        rightSideChecked = true;
        rgb.convertToHSV();
        return rgb.beaconSide();
    }

    public Alliance checkLeftSide(){
        float targetX = 0.0f;
        if (rightSideChecked && targeted()){
            float distance = 8.0f; //todo put in config
            targetX = (float)cameraVision.getX() + distance;
        }
        else if (!rightSideChecked) targetX = (float)3.0; //todo put number in config
        drive.startVelocity();
        while (cameraVision.getX() < targetX && targeted() && linearOpMode.opModeIsActive()){
            float theta = 0.0f;
            if (!rightSideChecked){
                theta = cameraVision.getX() > targetX ? (float)Math.toRadians(180.0) : 0.0f;
            }
            driveVector.setPolar(0.3f, theta);//todo put power in config
        }
        rgb.convertToHSV();
        return rgb.beaconSide();
    }

    public void press(){
        long waitTime = 1500; //todo put in config file
        long lastTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - lastTime < waitTime && linearOpMode.opModeIsActive()){
            driveVector.setPolar(0.3f, (float)Math.toRadians(270.0));
        }
    }

    public void chooseSide(RGB.Direction direction){
        float distance = Math.abs(180.0f); //todo put in config file
        float targetX = direction==LEFT ? distance : -distance;
        float theta = direction==LEFT ? 0.0f : (float)Math.toRadians(180.0);
        float radius = 0.3f;
        float margin = 10.0f; //todo configure
        drive.startVelocity();
        while (targeted() && linearOpMode.opModeIsActive() &&
                Math.abs(targetX-cameraVision.getX()) > margin){
            driveVector.setPolar(radius, theta);
        }
        driveVector.setPolar(0.0f, 0.0f);
    }

    /**
     *
     */
    public void reverse(){
        float theta = (float)Math.toRadians(90.0);
        float power = 0.3f;//todo config
        float distance = 800.9f;//todo config
        float waitTime = 1500f;
        float lastTime = System.currentTimeMillis();
        if (targeted()){
            while (targeted() && linearOpMode.opModeIsActive() &&
                    Math.abs(cameraVision.getZ()) < distance){
                driveVector.setPolar(power, theta);
            }
        }
        else while(linearOpMode.opModeIsActive() && System.currentTimeMillis()
                - lastTime < waitTime){
            driveVector.setPolar(power, theta);
        }
        driveVector.setPolar(0.0f, 0.0f);
    }

    /**
     *
     */
    public void updateCoordinates(){
        if (targeted()){
            Coordinate coordinate = cameraVision.updateCoordinates();
            drive.initPosition(coordinate.getX(), coordinate.getY());
        }
    }
}
