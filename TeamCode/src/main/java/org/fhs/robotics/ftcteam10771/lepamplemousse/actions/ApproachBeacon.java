package org.fhs.robotics.ftcteam10771.lepamplemousse.actions;

import com.qualcomm.robotcore.eventloop.SyncdDevice;
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
import static org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.RGB.Direction.NEITHER;

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
    private boolean rightSide = false;
    private boolean leftSide = false;
    boolean left = true;

    public ApproachBeacon(Config.ParsedData settings, Drive drive, LinearOpMode linearOpMode){
        this.drive = drive;
        this.settings = settings;
        this.driveVector = drive.getVectorR();
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;

        alliance = (settings.getString("alliance").equals("red")) ? Alliance.RED_ALLIANCE : Alliance.BLUE_ALLIANCE;
        //rgb = new RGB(hardwareMap.colorSensor.get("color_sensor_left"), hardwareMap.colorSensor.get("color_sensor_right"));
        rgb = new RGB(hardwareMap.colorSensor.get("left_rgb"),
                hardwareMap.led.get("left_led"), settings);
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

    public void press(){
        long waitTime = settings.subData("beacon").getInt("press_time"); //todo put in config file
        long lastTime = System.currentTimeMillis();
        float radius = settings.subData("beacon").getFloat("press_power");
        drive.startVelocity();
        if ((leftSide || rightSide) && leftSide!=rightSide){
            while (System.currentTimeMillis() - lastTime < waitTime && linearOpMode.opModeIsActive()){
                driveVector.setPolar(radius, (float)Math.toRadians(270.0));
                linearOpMode.telemetry.addData("Beacon", "pressing");
                linearOpMode.telemetry.update();
            }
        }
        driveVector.setPolar(0.0f, 0.0f);
        reverse();
    }

    /**
     * Suggested settings:
     * Saturation: 0.0
     * Value: 0.0
     * HueRed: 300-400
     * HueBlue: 200-300
     * @param left
     */
    public void chooseSide(boolean left){
        this.left = left;
        Alliance beaconAlliance = rgb.beaconSide();
        long lastTime = 0;
        long waitTime = 0;
        int sidesChecked = 0;
        float theta = left ? 0.0f : (float)Math.toRadians(180.0);
        float radius = settings.subData("beacon").getFloat("shift_power");
        drive.startVelocity();
        lastTime = System.currentTimeMillis();
        long maxCheckTime = settings.subData("beacon").getInt("max_check_time");
        while (linearOpMode.opModeIsActive() && beaconAlliance.equalsAlliance(Alliance.UNKNOWN)
                && System.currentTimeMillis() - lastTime < maxCheckTime){
            driveVector.setPolar(radius, theta);
            beaconAlliance = rgb.beaconSide();
        }
        driveVector.setPolar(0.0f, 0.0f);
        lastTime = System.currentTimeMillis();
        waitTime = settings.subData("beacon").getInt("check_time");
        while  (linearOpMode.opModeIsActive() && System.currentTimeMillis() - lastTime < waitTime){
            beaconAlliance = rgb.beaconSide();
        }
        if (alliance.equalsAlliance(Alliance.UNKNOWN)){
            leftSide = false;
            rightSide = false;
            return;
        }
        if (alliance.equalsAlliance(beaconAlliance)){
            while (linearOpMode.opModeIsActive() && beaconAlliance.equalsAlliance(alliance)){
                driveVector.setPolar(radius, theta);
                beaconAlliance = rgb.beaconSide();
            }
            driveVector.setPolar(0,0);
            lastTime = System.currentTimeMillis();
            while (linearOpMode.opModeIsActive() && System.currentTimeMillis() - lastTime < waitTime){
                beaconAlliance = rgb.beaconSide();
            }
            if (!beaconAlliance.equals(Alliance.UNKNOWN)){
                theta = left ? (float)Math.toRadians(180.0) : 0.0f;
                while(linearOpMode.opModeIsActive() && !alliance.equalsAlliance(beaconAlliance)){
                    driveVector.setPolar(radius, theta);
                }
                driveVector.setPolar(0f, 0f);
                if (left){
                    waitTime = settings.subData("beacon").getInt("shift_time");
                    radius = settings.subData("beacon").getFloat("fast_power");
                    lastTime = System.currentTimeMillis();
                    while(System.currentTimeMillis() - lastTime < waitTime && linearOpMode.opModeIsActive()){
                        driveVector.setPolar(radius, theta);
                    }
                    rightSide = true;
                    leftSide = false;
                }
                else {
                    rightSide = false;
                    leftSide = true;
                }
            }
            else{
                rightSide = true;
                leftSide = true;
            }
        }
        else while(linearOpMode.opModeIsActive() && !(rgb.beaconSide().equals(alliance) || rgb.beaconSide().equals(Alliance.UNKNOWN))){
            driveVector.setPolar(radius, theta);
            if (!left){
                waitTime = settings.subData("beacon").getInt("shift_time");
                radius = settings.subData("beacon").getFloat("fast_power");
                lastTime = System.currentTimeMillis();
                while(System.currentTimeMillis() - lastTime < waitTime && linearOpMode.opModeIsActive()){
                    driveVector.setPolar(radius, theta);
                }
                rightSide = true;
                leftSide = false;
            }
            else {
                rightSide = false;
                leftSide = true;
            }
        }
        driveVector.setPolar(0.0f, 0.0f);
    }

    /**
     *
     */
    public void reverse(){
        float theta = (float)Math.toRadians(90.0);
        float power = 0.3f;//todo config
        long waitTime = settings.subData("beacon").getInt("reverse_time");
        long lastTime = System.currentTimeMillis();
        while(linearOpMode.opModeIsActive() && System.currentTimeMillis()
                - lastTime < waitTime){
            driveVector.setPolar(power, theta);
        }
        driveVector.setPolar(0.0f, 0.0f);
    }

    public void adjustAfterClaim(){
        driveVector.setPolar(0f, 0f);
        drive.resetEncoders();
        final float offset = 22.52f;
        boolean pressed = true;
        if (rightSide && !leftSide){
            drive.initPosition(-offset, 0f);
        }
        else if (leftSide && !rightSide){
            drive.initPosition(offset, 0f);
        }
        else {
            pressed = false;
        }
        if (!pressed){
            if (left){
                drive.initPosition(30f, 0);
            }
            else drive.initPosition(-30f, 0);
        }
        drive.startPosition();
        driveVector.setX(0f);
        driveVector.setY(61.4f);
        boolean proceed = false;
        while(!proceed){
            pressed = drive.isAtPosition();
        }
        drive.startPosition();
        driveVector.setPolar(0f, 0f);
    }

    /**
     *
     */
    public void updateCoordinates(){
        boolean found = false;
        long waitTime = 4000;
        lastTime = System.currentTimeMillis();
        while (!found && System.currentTimeMillis() - lastTime < waitTime){
            found = targeted();
        }
        if (targeted()){
            Coordinate coordinate = cameraVision.updateCoordinates();
            drive.initPosition(coordinate.getX(), coordinate.getY());
            drive.resetEncoders();
        }
        cameraVision().stop();
    }

    public String beaconAlliance(){
        return rgb.beaconSide().getAlliance();
    }

    /**
     * If using this,
     * red hue: 300-400;
     * blue hue: 200-300;
     * value: 0
     * saturation: 0
     */
    public void defaultChoose(){
        Alliance beacon;
        long lastTime = 0;
        long waitTime = 0;
        float theta = 0.0f;
        float radius = settings.subData("beacon").getFloat("shift_power");
        drive.startVelocity();
        beacon = rgb.beaconSide();
        while (linearOpMode.opModeIsActive() && beacon.equals(Alliance.UNKNOWN)){
            driveVector.setPolar(radius, theta);
            beacon = rgb.beaconSide();
            linearOpMode.telemetry.addData("Beacon", "unknown");
            linearOpMode.telemetry.update();
        }
        lastTime = System.currentTimeMillis();
        waitTime = settings.subData("beacon").getInt("check_time");
        while  (linearOpMode.opModeIsActive() && System.currentTimeMillis() - lastTime < waitTime){
            driveVector.setPolar(0.0f, 0.0f);
            beacon = rgb.beaconSide();
        }
        if (alliance.equals(beacon)){
            waitTime = settings.subData("beacon").getInt("shift_time");
            theta = (float)Math.toRadians(180.0);
            radius = settings.subData("beacon").getFloat("power");
            lastTime = System.currentTimeMillis();
            while(System.currentTimeMillis() - lastTime < waitTime && linearOpMode.opModeIsActive()){
                driveVector.setPolar(radius, theta);
                linearOpMode.telemetry.addData("Beacon", "correct");
                linearOpMode.telemetry.update();
            }
            rightSide = true;
            leftSide = false;
        }
        else {
            while(linearOpMode.opModeIsActive() && !(beacon.equals(alliance) || beacon.equals(Alliance.UNKNOWN))){
                driveVector.setPolar(radius, theta);
                beacon = rgb.beaconSide();
                linearOpMode.telemetry.addData("Beacon", "incorrect");
                linearOpMode.telemetry.update();
            }
            if (!beacon.equalsAlliance(Alliance.UNKNOWN)) leftSide = true;
            rightSide = false;
        }
        driveVector.setPolar(0.0f, 0.0f);
    }

    public void claimBeacon(){
        long wait = settings.subData("beacon").getInt("offset_time");
        long last = System.currentTimeMillis();
        float theta = (float) Math.toRadians(180.0);
        float radius = settings.subData("beacon").getFloat("fast_power");
        drive.startVelocity();
        while (System.currentTimeMillis() - last < wait && linearOpMode.opModeIsActive()){
            driveVector.setPolar(radius, theta);
        }
        driveVector.setPolar(0f, 0f);
        defaultChoose();
        //rotate(180.0, false);
        press();
    }

    public String beaconStat(){
        if (leftSide && !rightSide) return "left";
        if (rightSide && !leftSide) return "right";
        if (!leftSide && !rightSide) return "neither";
        else return "both";
    }
}
