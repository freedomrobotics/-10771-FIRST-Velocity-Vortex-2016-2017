package org.fhs.robotics.ftcteam10771.lepamplemousse.modes.disabled;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.Drive;
import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Alliance;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Components;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.components.Aliases;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.IMU;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.RGB;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.CameraVision;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.Static;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Coordinate;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Rotation;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.entities.Robot;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector.VectorR;

import java.util.List;

import static org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.IMU.Axis.Z;
import static org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.RGB.Direction.LEFT;
import static org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.RGB.Direction.RIGHT;

/**
 * Created by joelv on 2/9/2017.
 */
@Disabled
@Autonomous (name="Pampletonomous")
public class PamplemouseAuto extends LinearOpMode {

    private Config rawSettings;
    private Config.ParsedData settings;
    Config.ParsedData parsedField;
    private Config fieldMapConfig;
    private Components components;
    private Drive drive;
    private CameraVision cameraVision;
    private RGB rgb;
    private IMU imuHandler;
    private IMU.Gyrometer gyroOutput;
    private VectorR driveVector = new VectorR(new Coordinate(), new Rotation());
    private boolean backCamera = true;
    String team;
    Alliance alliance;
    private float distance_to_stop = 500.0f;
    private boolean proceed = false;

    Runnable runnable = new Runnable() {
        @Override
        public void run() {
            while (!Thread.currentThread().isInterrupted()) {
                telemetry.addData("Team", team);
                telemetry.addData("cameraZ", cameraVision.getZ());
                telemetry.addData("FR", drive.getMotorPower(1));
                telemetry.addData("FL", drive.getMotorPower(2));
                telemetry.addData("BL", drive.getMotorPower(3));
                telemetry.addData("BR", drive.getMotorPower(4));
                telemetry.addData("FR", drive.getEncoder(1));
                telemetry.addData("FL", drive.getEncoder(2));
                telemetry.addData("BL", drive.getEncoder(3));
                telemetry.addData("BR", drive.getEncoder(4));
                telemetry.addData("Distance", distance_to_stop);
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

        fieldMapConfig = new Config("/Position", "fieldmap.yml", telemetry, "field");
        if (fieldMapConfig.read()== Config.State.DEFAULT_EXISTS){
            fieldMapConfig.create(true);
            if (fieldMapConfig.read()== Config.State.DEFAULT_EXISTS)
                fieldMapConfig.read(true);
        }

        this.settings = rawSettings.getParsedData();

        this.components = new Components(hardwareMap, telemetry, components);
        this.components.initialize();

        Config.ParsedData drivetrainMotors = settings.subData("drivetrain").subData("motor");

        team = settings.getString("alliance");
        alliance = (team.equals("red")) ? Alliance.RED_ALLIANCE : Alliance.BLUE_ALLIANCE;
        parsedField = fieldMapConfig.getParsedData().subData("coordinates").subData(team);

        cameraVision = new CameraVision();
        //todo complete framework_test configuration
        rgb = new RGB(hardwareMap.colorSensor.get("left_rgb"), hardwareMap.colorSensor.get("right_rgb"),
                hardwareMap.led.get("left_led"), hardwareMap.led.get("right_led"));
        drive = new Drive(driveVector, new Robot(),
                Aliases.motorMap.get(drivetrainMotors.subData("front_right").getString("map_name")),
                Aliases.motorMap.get(drivetrainMotors.subData("front_left").getString("map_name")),
                Aliases.motorMap.get(drivetrainMotors.subData("back_left").getString("map_name")),
                Aliases.motorMap.get(drivetrainMotors.subData("back_right").getString("map_name")),
                settings, telemetry);
        imuHandler = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
        gyroOutput = imuHandler.getGyrometer();
        telemetryThread.start();
        cameraVision.cameraThread.start();
        waitForStart();
        imuHandler.imuThread.start();
        drive.setRelative(true);
        drive.startPosition();
        wait(settings.subData("drive").getInt("wait_time"));
        if (!parsedField.getString("beacon1").equals("null")){
            claimBeacon(parsedField.getString("beacon1"));
        }
        if (!parsedField.getString("beacon2").equals("null")){
            claimBeacon(parsedField.getString("beacon2"));
        }
        driveVector.setX(drive.getCurrentX());
        driveVector.setY(drive.getCurrentY());
        drive.startPosition();
        cameraVision.cameraThread.interrupt();
        /*
        if (parsedField.getBool("knock_ball")){
            driveTo("center");
            while(!proceed){
                proceed = drive.isAtPosition();
            }
        }
        driveTo(parsedField.getString("park"));
        proceed = false;
        while(!proceed){
            proceed = drive.isAtPosition();
        }
        driveVector.setX(drive.getCurrentX());
        driveVector.setY(drive.getCurrentY());
        *///todo complete imu code before making uncommenting
        imuHandler.imuThread.interrupt();
        telemetryThread.interrupt();
        drive.stop();
        //add the imu and possibly catapult if there is time
    }

    private void claimBeacon(String beacon){
        drive.startPosition();
        driveTo(beacon);
        while(!proceed){
            proceed = drive.isAtPosition();
        }
        driveVector.setPolar(0.0f, 0.0f);//fixme this might be the problem
        drive.startVelocity();
        proceed = false;
        //todo rotate accordingly
        long currentTime = System.currentTimeMillis();
        int waitTime = settings.subData("drive").subData("camera_settings").getInt("detect_time");
        //todo put in config file
        boolean detected = false;
        while(System.currentTimeMillis()-currentTime<waitTime && !detected){
            detected = cameraVision.countTrackedImages()==1;
        }
        if (detected){
            if (cameraVision.imageInSight()){centerRotate();}
            if (cameraVision.imageInSight()){approach();}
            if (checkBeaconSide(LEFT)){
                rgb.indicateCorrcect(LEFT, alliance);
                press();
            }
            else if (checkBeaconSide(RIGHT)){
                rgb.indicateCorrcect(RIGHT, alliance);
                press();
            }
        }
    }

    /**
     * Center towards the image
     */
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
        driveVector.setRadius(0);
        driveVector.setPolar(0, 0);
    }

    /**
     * Approaches the image
     */
    private void approach() {
        distance_to_stop = settings.subData("drive").subData("camera_settings").getFloat("distance_to_stop");
        if (targeted()) {
            while (targeted() && Math.abs(cameraVision.getZ()) > distance_to_stop) {
                float theta = backCamera ? 3.0f * (float) Math.PI / 2.0f : (float) Math.PI / 2.0f;
                float radius = settings.subData("drive").subData("camera_settings").getFloat("speed");
                //todo put speed
                driveVector.setPolar(radius, theta);
            }
        }
        driveVector.setRadius(0);
        driveVector.setPolar(0, 0);
    }


    /**
     * Rotates the robot to face the wall
     */
    public void rotateToWall(){
        float rotate_margin = (float)Math.toRadians(settings.subData("drive").subData("camera_settings").getFloat("angle_margin"));
        float rotate_speed = settings.subData("drive").subData("camera_settings").getFloat("rotate_speed");
        if (targeted()){
            while(targeted() && Math.toDegrees(Math.abs(cameraVision.getAngleToTurn()))>rotate_margin){
                driveVector.setRad((float)Math.copySign(rotate_speed, cameraVision.getAngleToTurn()));
            }
        }
        driveVector.setRadius(0);
        driveVector.setPolar(0, 0);
    }

    /**
     * Rotate to IMU
     * @param radians
     */
    public void rotate(float radians){
        if (drive.isVectorDriveActive()){
            float rotate_margin = (float)Math.toRadians(settings.subData("drive").subData("camera_settings").getFloat("angle_margin"));
            float rotate_speed = settings.subData("drive").subData("camera_settings").getFloat("rotate_speed");
            float orientation = gyroOutput.convert(Z, gyroOutput.getOrientation(Z));
            while(orientation  < radians-rotate_margin || orientation > radians+rotate_margin){
                orientation = gyroOutput.convert(Z, gyroOutput.getOrientation(Z));
                driveVector.setRad((float)Math.copySign(rotate_speed, radians - orientation));
            }
        }
        else {
            driveVector.setRad(radians);
            proceed = false;
            while(!proceed){
                float rotate_margin = (float)Math.toRadians(settings.subData("drive").subData("camera_settings").getFloat("angle_margin"));
                float orientation = gyroOutput.convert(Z, gyroOutput.getOrientation(Z));
                proceed = (orientation > radians-rotate_margin && orientation < orientation - rotate_margin);
            }
            driveVector.setRad(gyroOutput.convert(Z, gyroOutput.getOrientation(Z)));
        }
        //drive.refresh();
    }

    /**
     * Centers and rotates towards the image
     */
    private void centerRotate(){
        float rotate_margin = (float) Math.toRadians(settings.subData("drive").subData("camera_settings").getFloat("angle_margin"));
        float rotate_speed = settings.subData("drive").subData("camera_settings").getFloat("rotate_speed");
        float marginofError = settings.subData("drive").subData("camera_settings").getFloat("centering_margin");
        float radius = settings.subData("drive").subData("camera_settings").getFloat("speed");
        float rotate_factor = settings.subData("drive").subData("camera_settings").getFloat("rotate_factor");
        while (targeted() && Math.abs(cameraVision.getX()) > marginofError || Math.abs(cameraVision.getAngleToTurn())>rotate_margin) {
            boolean left = backCamera ? (cameraVision.getX() > 0.0) : (cameraVision.getX() < 0.0);
            float rotate = (float)(radius/(rotate_factor*rotate_speed) * cameraVision.getAngleToTurn());
            float side = (float)((rotate_factor*rotate_speed)/radius * cameraVision.getX());
            float theta = left ? (float) Math.PI : 0.0f;
            driveVector.setPolar(radius, theta);
            driveVector.setRad((float)Math.copySign(rotate, cameraVision.getAngleToTurn()));
        }
        driveVector.setRadius(0);
        driveVector.setPolar(0, 0);
    }

    /**
     * Check a beacon side
     * @param direction
     * @return
     */
    private boolean checkBeaconSide(RGB.Direction direction){
        if (targeted()){
            float distance = settings.subData("drive").subData("camera_settings").getFloat("beacon_check_distance");
            float radius = settings.subData("drive").subData("camera_settings").getFloat("slow_speed");
            //todo add slow_speed to config file
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
            distance = direction== LEFT ? Math.abs(distance) : -Math.abs(distance);
            if (direction== LEFT){
                while(targeted() && cameraVision.getX()<distance){
                    driveVector.setPolar(radius, theta);
                }
            }
            else if (direction== RIGHT){
                while(targeted() && cameraVision.getX()>distance){
                    driveVector.setPolar(radius, theta);
                }
            }
            driveVector.setPolar(0.0f, 0.0f);
            return rgb.isSide(alliance, direction);
        }
        else return false;
    }

    private void press(){
        float radius = settings.subData("drive").subData("camera_settings").getFloat("slow_speed");
        float theta = (float)Math.PI * 3.0f / 2.0f;

        int waitTime = settings.subData("drive").subData("camera_settings").getInt("wait_time");
        long currentTime = System.currentTimeMillis();
        while(System.currentTimeMillis()-currentTime<waitTime){
            driveVector.setPolar(radius, theta);
        }
        driveVector.setPolar(0.0f, 0.0f);
    }

    private boolean targeted(){
        return cameraVision.imageInSight(cameraVision.target());
    }

    private void driveTo(String location){
        setCoordinate(location);
    }
    private void setCoordinate(String location){
        driveVector.setX(parsedField.subData(location).getFloat("x"));
        driveVector.setY(parsedField.subData(location).getFloat("y"));
    }
}
