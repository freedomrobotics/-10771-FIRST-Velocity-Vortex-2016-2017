package org.fhs.robotics.ftcteam10771.lepamplemousse.actions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Alliance;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.RGB;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.CameraVision;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector.VectorR;

import static org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.RGB.Direction.BOTH;

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
}
