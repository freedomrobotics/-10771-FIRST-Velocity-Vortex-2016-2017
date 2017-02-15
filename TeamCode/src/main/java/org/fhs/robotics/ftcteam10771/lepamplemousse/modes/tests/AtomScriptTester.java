package org.fhs.robotics.ftcteam10771.lepamplemousse.modes.tests;

import android.speech.tts.TextToSpeech;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.Drive;
import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.scriptedconfig.ScriptLoader;
import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.scriptedconfig.ScriptRunner;
import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Alliance;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Components;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Controllers;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.components.Aliases;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.mechanisms.Catapult;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.IMU;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.CameraVision;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.Static;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.entities.Robot;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector.VectorR;

import java.util.List;

import static org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.IMU.Axis.Z;

/**
 * Created by Freedom Robotics on 2/10/2017.
 */
@Autonomous(name= "scripted")
public class AtomScriptTester extends LinearOpMode implements ScriptRunner, TextToSpeech.OnInitListener {
    Controllers controls;
    private long lastTime;      // The time at the last time check (using System.currentTimeMillis())
    private Config fieldMapConfig;
    private Config rawSettings;
    private Config.ParsedData settings;
    private Config.ParsedData fieldMap;
    private Components components;
    private Robot robot;
    private Drive drive;
    private VectorR driveVector = new VectorR();
    private IMU imuHandler;
    private IMU.Gyrometer gyrometer;
    private Catapult catapult;
    private boolean dropBalls = false;
    private Servo ballDropper;
    private DcMotor intakeMotor;
    private DcMotor launcher;
    private boolean status;
    private TextToSpeech speech;
    private String team = "blue";
    private CameraVision cameraVision;

    @Override
    public void runOpMode() throws InterruptedException {
        Config keymapping = new Config(Static.configPath, Static.configControlFileName + Static.configFileSufffix, telemetry, "keymapping");
        // should catch other errors, but oh well
        // add data logging
        if (keymapping.read() == Config.State.DEFAULT_EXISTS) {
            keymapping.create(true);
            //read without creating file if it still fails.
            if (keymapping.read() == Config.State.DEFAULT_EXISTS)
                keymapping.read(true);
        }

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

        fieldMapConfig = new Config("/Position", "fieldmap.yml", telemetry, "field");
        if (fieldMapConfig.read()== Config.State.DEFAULT_EXISTS){
            fieldMapConfig.create(true);
            if (fieldMapConfig.read()== Config.State.DEFAULT_EXISTS)
                fieldMapConfig.read(true);
        }

        this.fieldMap = fieldMapConfig.getParsedData();

        // TODO: 2/10/2017 Initialize for realz
        team = this.settings.getString("alliance");
        robot = new Robot();

        this.components = new Components(hardwareMap, telemetry, components);
        this.components.initialize();
        this.controls = new Controllers(gamepad1, gamepad2, keymapping);
        this.controls.initialize();

        //setup drive thread
        Config.ParsedData drivetrainMotors = settings.subData("drivetrain").subData("motor");

        drive = new Drive(driveVector, new Robot(),
                Aliases.motorMap.get(drivetrainMotors.subData("front_right").getString("map_name")),
                Aliases.motorMap.get(drivetrainMotors.subData("front_left").getString("map_name")),
                Aliases.motorMap.get(drivetrainMotors.subData("back_left").getString("map_name")),
                Aliases.motorMap.get(drivetrainMotors.subData("back_right").getString("map_name")),
                settings, telemetry);

        //PREP THE COMMAND LIST!
        if (settings.subData("autonomous").getObject("command_list") == null)
            return;

        //setup imu
        imuHandler = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
        imuHandler.imuInit(); //todo remember to init imu
        gyrometer = imuHandler.getGyrometer();
        gyrometer.enableStream(true);
        imuHandler.setStreamDelay(150);
        imuHandler.imuThread.start();

        //setup camera

        Config.ParsedData bumpers = settings.subData("bumper");

        Servo bumperLeft = Aliases.servoMap.get(bumpers.subData("left_servo").getString("map_name"));
        Servo bumperRight = Aliases.servoMap.get(bumpers.subData("right_servo").getString("map_name"));
        if (bumpers.subData("left_servo").getBool("reversed"))
            bumperLeft.setDirection(Servo.Direction.REVERSE);
        if (bumpers.subData("right_servo").getBool("reversed"))
            bumperRight.setDirection(Servo.Direction.REVERSE);
        float bumperRange = bumpers.getFloat("full_rotate");
        bumperLeft.setPosition(bumpers.subData("left_servo").getFloat("offset") / bumperRange);
        bumperRight.setPosition(bumpers.subData("right_servo").getFloat("offset") / bumperRange);
        //rgb = new RGB(hardwareMap.colorSensor.get("color_sensor_left"), hardwareMap.colorSensor.get("color_sensor_right"));

        //setup catapult
        launcher = hardwareMap.dcMotor.get(settings.subData("catapult").getString("map_name"));
        catapult = new Catapult(launcher, hardwareMap.opticalDistanceSensor.get("ods"), controls, settings);
        catapult.catapultThread.start();

        //setup ball dropper

        intakeMotor = hardwareMap.dcMotor.get("motorIntake");
        ballDropper = hardwareMap.servo.get("drop");
        if (settings.subData("drop").getBool("reversed")){
            ballDropper.setDirection(Servo.Direction.REVERSE);
        } else{
            ballDropper.setDirection(Servo.Direction.FORWARD);
        }
        dropBalls();

        //setup command list
        List<String> commandList = (List<String>) settings.subData("autonomous").getObject("command_list");

        //text to speech
        speech = new TextToSpeech(hardwareMap.appContext, this);

        int counter = 1; //debug
        //WAIT FOR THE STARTI!@HTIOgRFOIWUQ#GQIOH
        telemetry.addData("ready", true);
        telemetry.addData("orient", gyrometer.getOrientation(IMU.Axis.Z));
        telemetry.update();
        waitForStart();
        for(String command : commandList){
            //check if opmode is active
            if (!opModeIsActive()) break;
            this.lastTime = System.currentTimeMillis();

            ScriptLoader.CommandParser commandParser = new ScriptLoader.CommandParser(command);

            commandPicker(commandParser);

            if (commandParser.command().equalsIgnoreCase("stop")){
                break;
            }

            //Debug section
            telemetry.addData("robot", robot.toString());
            telemetry.addData(counter + "", command);
            counter++;
            telemetry.update();
        }
        drive.stop();
        imuHandler.imuThread.interrupt();
        catapult.catapultThread.interrupt();
        //speech.shutdown();
    }

    @Override
    public void commandPicker(ScriptLoader.CommandParser commandParser) throws InterruptedException {

        if (commandParser.command().equalsIgnoreCase("move_time")){
            if (commandParser.getArgsSize() == 2){
                drive.startVelocity();
                drive.setRelative(true);
                driveVector.setPolar(commandParser.getArgFloat(0), (float)(3.0/2.0*Math.PI));
                sleep(commandParser.getArgInt(1));
                driveVector.setPolar(0, 0);
                //define function for a 2 argument command
                //drive(commandParser.getArgFloat(0), commandParser.getArgFloat(1));

            } if(commandParser.getArgsSize() >= 3){
                drive.startVelocity();
                drive.setRelative(true);
                driveVector.setPolar(commandParser.getArgFloat(0), (float)Math.toRadians(commandParser.getArgFloat(1)));
                sleep(commandParser.getArgInt(2));
                driveVector.setPolar(0, 0);
                //move_position 3.5,3,500
            }
            return;
        }


        if (commandParser.command().equalsIgnoreCase("approach_beacon")){
            drive.setRelative(true);
            drive.startVelocity();
            //drive.driveThread.start();
            //rotate();
            //center();
            /*
            while (!targeted() && System.currentTimeMillis() - lastTime < commandParser.getArgInt(0) && opModeIsActive()){}
            if (!targeted())*/
               // return;
            //centerRotate();
            //approach();
            //
            driveVector.setPolar(0, 0);
            return;
        }

        if (commandParser.command().equalsIgnoreCase("rotate")){
            rotate(commandParser.getArgFloat(0));
            return;
        }

        if (commandParser.command().equalsIgnoreCase("catapult")) {
            catapult.setLaunch(true);
            sleep(5);
            catapult.setLaunch(false);
            sleep(300);
            while (!catapult.catapultReady() && opModeIsActive());
            return;
        }

        if (commandParser.command().equalsIgnoreCase("tts")) {
            while (!status && opModeIsActive()){
            }
            speech.playEarcon(commandParser.getArgString(0), TextToSpeech.QUEUE_ADD, null);
            return;
        }

        if (commandParser.command().equalsIgnoreCase("ball_drop")){
            dropBalls = commandParser.getArgString(0).equalsIgnoreCase("up");
            dropBalls();
            return;
        }

        if (commandParser.command().equalsIgnoreCase("intake")){
            intakeMotor.setPower(commandParser.getArgFloat(0));
        }

        if (commandParser.command().equalsIgnoreCase("pause")){
            sleep(commandParser.getArgInt(0));
            return;
        }

        if (settings.subData("autonomous").getObject(commandParser.command()) == null)
            return;

        List<String> commandList = (List<String>) settings.subData("autonomous").getObject(commandParser.command());
        for(String command : commandList) {
            if (!opModeIsActive()) break;

            ScriptLoader.CommandParser internalScript = new ScriptLoader.CommandParser(command);

            commandPicker(internalScript);

            if (internalScript.command().equalsIgnoreCase("stop")){
                return;
            }
        }
    }

    /**
     * Rotates the robot to a target Z orientation using the IMU
     * todo test rotate function using script
     * @param degrees the angular orientation to rotate to around the Z axis
     */
    public void rotate(double degrees){
        drive.startVelocity();
        drive.setRelative(true);
        final double fullRotation = Math.toRadians(360.0);
        double targetOrientation = Math.toRadians(degrees);
        double orientation = fullRotation + gyrometer.getOrientation(Z);
        double rotate_margin = Math.toRadians(settings.subData("drive").subData("camera_settings").getFloat("angle_margin"));
        float rotate_speed = settings.subData("drive").subData("camera_settings").getFloat("rotate_speed");
        boolean targetNearZero = (targetOrientation < rotate_margin || targetOrientation > fullRotation - rotate_margin);
        if (targetOrientation > fullRotation - rotate_margin) targetOrientation -= fullRotation;
        while(orientation < targetOrientation - rotate_margin
                || orientation > targetOrientation + rotate_margin && opModeIsActive()){
            if (targetNearZero){
                orientation = gyrometer.convertAngletoSemiPossibleRange(
                        Z, gyrometer.getOrientation(Z));
            }
            else orientation = Math.toRadians(360.0) + gyrometer.getOrientation(Z);
            telemetry.addData("Orientation", gyrometer.getOrientation(Z));
            driveVector.setRad((float)Math.copySign(rotate_speed, targetOrientation - orientation));
            telemetry.update();
        }
        telemetry.addData("Rotation", "Done");
        telemetry.update();
        driveVector.setRad(0);
        driveVector.setPolar(0, 0);
    }


    /**
     * Positionally drive the robot to a location on the field
     * Unit in cm, and using FTC field coordinate system
     * @param x coordinate to drive to
     * @param y coordinate to drive to
     */
    public void driveTo(float x, float y){
        rotate(0.0); //todo comment if imu is finished
        drive.startPosition();
        //todo make a config file for margins and such
        float margin = Math.abs(settings.subData("drive").subData("camera_settings").getFloat("distance_to_stop"));
        float xDistance = drive.getCurrentX() - x;
        float yDistance = drive.getCurrentY() - y;
        float distance = (float)Math.sqrt((xDistance*xDistance)+(yDistance*yDistance));
        driveVector.setX(x);
        driveVector.setY(y);
        while (distance > margin){
            xDistance = drive.getCurrentX() - x;
            yDistance = drive.getCurrentY() - y;
            distance = (float)Math.sqrt((xDistance*xDistance)+(yDistance*yDistance));
        }
        driveVector.setX(drive.getCurrentX());
        driveVector.setY(drive.getCurrentY());
    }

    /**
     * The drive to center vortex
     */
    public void driveToCenterVortex(float power){
        rotate(0.0); //todo comment after imu completion
        final float fullRotation = (float)Math.toRadians(360.0);
        float vortexX = fieldMap.subData("coordinates").subData(team).subData("center_vortex").getFloat("x");
        float vortexY = fieldMap.subData("coordinates").subData(team).subData("center_vortex").getFloat("y");
        float xDistance = drive.getCurrentX() - vortexX;
        float yDistance = drive.getCurrentY() - vortexY;
        float distance = (float)Math.sqrt((xDistance*xDistance)+(yDistance*yDistance));
        float theta = (float)Math.atan(yDistance/xDistance);
        float targetDistance = fieldMap.subData("values").getFloat("distance_from_vortex");
        float margin = 3.0f; //todo move all margins under new heading called margins:
        drive.startVelocity();
        while (distance < targetDistance - margin || distance > targetDistance + margin){
            if (distance > targetDistance) theta = (theta + fullRotation/2.0f) % fullRotation;
            driveVector.setPolar(power, theta);
        }
        rotate((theta + fullRotation/2.0f) % fullRotation);
    }

    /*
    public void centerRotate(){
        float rotate_margin = (float) Math.toRadians(settings.subData("drive").subData("camera_settings").getFloat("angle_margin"));
        float rotate_speed = settings.subData("drive").subData("camera_settings").getFloat("rotate_speed");
        float marginofError = settings.subData("drive").subData("camera_settings").getFloat("centering_margin");
        float radius = settings.subData("drive").subData("camera_settings").getFloat("speed");
        float rotate_factor = settings.subData("drive").subData("camera_settings").getFloat("rotate_factor");
        if (targeted()) {
            while (Math.abs(cameraVision.getX()) > marginofError || Math.abs(cameraVision.getAngleToTurn())>rotate_margin) {
                boolean left = backCamera ? (cameraVision.getX() > 0.0) : (cameraVision.getX() < 0.0);
                float rotate = (float)(radius/(rotate_factor*rotate_speed) * cameraVision.getAngleToTurn());
                float side = (float)((rotate_factor*rotate_speed)/radius * cameraVision.getX());
                float theta = left ? (float) Math.PI : 0.0f;
                driveVector.setPolar(radius, theta);
                driveVector.setRad((float)Math.copySign(rotate, cameraVision.getAngleToTurn()));
            }
        }
        driveVector.setRadius(0);
        driveVector.setPolar(0, 0);
    }

    public void approach() {
        float distance_to_stop = settings.subData("drive").subData("camera_settings").getFloat("distance_to_stop");
        if (targeted()) {
            while (Math.abs(cameraVision.getZ()) > distance_to_stop) {
                float theta = backCamera ? 3.0f * (float) Math.PI / 2.0f : (float) Math.PI / 2.0f;
                float radius = settings.subData("drive").subData("camera_settings").getFloat("speed");
                //todo put speed
                driveVector.setPolar(radius, theta);
            }
        }
        driveVector.setRadius(0);
        driveVector.setPolar(0, 0);
    }
    private boolean targeted(){
        return cameraVision.imageInSight(cameraVision.target());
    }
    */

    /**
     * Lifts plow if it is down
     * Drops plow if it is lifted
     */
    public void dropBalls(){
        Config.ParsedData drop = settings.subData("drop");
        float fullRange = drop.getFloat("full_rotate");
        float offset = drop.getFloat("offset") / fullRange;
        float up = drop.getFloat("up_angle") / fullRange;
        float down = drop.getFloat("down_angle") / fullRange;
        if (dropBalls){
            ballDropper.setPosition(up + offset);
            telemetry.addData("dropper", "up");
        }else {
            ballDropper.setPosition(down + offset);
            telemetry.addData("dropper", "down");
        }
    }

    @Override
    public void onInit(int status) {
        this.status = status == TextToSpeech.SUCCESS;
    }
}