package org.fhs.robotics.ftcteam10771.lepamplemousse.modes.tests;

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

/**
 * Tests the camera drive class
 * Created by joelv on 2/3/2017.
 */
@Autonomous(name="Camera Drive", group="Test")
public class CameraDriveOp extends LinearOpMode{

    private long lastTime;
    private Config rawSettings;
    private Config.ParsedData settings;
    private Components components;
    private Drive drive;
    private CameraVision cameraVision;
    private RGB rgb;
    private VectorR driveVector = new VectorR(new Coordinate(), new Rotation());
    private boolean backCamera = true;
    Alliance alliance;

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
                settings, null, telemetry);

        //FIXME: I forgot how the aliases work - Joel
        alliance = (settings.getString("alliance")=="red") ? Alliance.RED_ALLIANCE : Alliance.BLUE_ALLIANCE;
        rgb = new RGB(hardwareMap.colorSensor.get("color_sensor_left"), hardwareMap.colorSensor.get("color_sensor_right"));

        cameraVision = new CameraVision();
        backCamera = cameraVision.usingBackCamera();
        cameraVision.cameraThread.start();

        this.lastTime = System.currentTimeMillis();

        waitForStart();

        drive.setRelative(true);
        drive.startVelocity();
        rotate();
        center();
        approach();
        /*
        if ((checkBeaconSide(RGB.Direction.LEFT))){
            telemetry.addData("Side", "left");
            telemetry.update();
        }
        else if (checkBeaconSide(RGB.Direction.RIGHT)){
            telemetry.addData("Side", "right");
            telemetry.update();
        }
        */
        idle();
    }

    public void center() {
        float marginofError = settings.subData("drive").subData("camera_settings").getFloat("centering_margin");
        if (targeted()){
            while(Math.abs(cameraVision.getX())>marginofError) {
                boolean left = backCamera ? (cameraVision.getX()>0.0) : (cameraVision.getX()<0.0);
                float theta = left ? (float)Math.PI : 0.0f;
                float radius = (Coordinate.convertTo((float) Math.abs(cameraVision.getX()), Coordinate.UNIT.MM_TO_UNIT));
                radius = (Coordinate.convertTo(radius, Coordinate.UNIT.UNIT_TO_DM));
                drive.setVelocity(radius, theta);
            }
            drive.stop();
        }
    }

    public void approach(){
        float distance_to_stop = settings.subData("drive").subData("camera_settings").getFloat("distance_to_stop");
        if (targeted()){
            while(Math.abs(cameraVision.getZ())>distance_to_stop){
                float theta = backCamera ? 3.0f*(float)Math.PI/2.0f : (float)Math.PI/2.0f;
                float radius = (Coordinate.convertTo((float) cameraVision.getZ(), Coordinate.UNIT.MM_TO_UNIT));
                //todo put power cutback in settings
                radius = 0.5f * (Coordinate.convertTo(radius, Coordinate.UNIT.UNIT_TO_M));
                drive.setVelocity(radius, theta);
            }
            drive.stop();
        }
    }

    public void rotate(){
        float rotate_margin = settings.subData("drive").subData("camera_settings").getFloat("angle_margin");
        if (targeted()){
            while(Math.abs(cameraVision.getAngleToTurn())>rotate_margin){
                drive.setRoation((float)cameraVision.getAngleToTurn());
            }
            drive.stop();
        }
    }

    public boolean checkBeaconSide(RGB.Direction direction){
        if (targeted()){
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
            while (Math.abs(cameraVision.getX())<distance){
                float radius = distance - (float)Math.abs(cameraVision.getX());
                //todo put power cutback ratio in settings config
                radius *= 0.08f;
                drive.setVelocity(radius, theta);
            }
            drive.stop();
            return rgb.isSide(alliance, direction);
        }
        else return false;
    }

    private boolean targeted(){
        return cameraVision.imageInSight(cameraVision.target());
    }
}
