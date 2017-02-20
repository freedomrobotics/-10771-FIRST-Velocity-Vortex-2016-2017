package org.fhs.robotics.ftcteam10771.lepamplemousse.modes.final_op_modes;

import android.util.Log;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.Drive;
import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Alliance;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Components;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Controllers;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.components.Aliases;
import org.fhs.robotics.ftcteam10771.lepamplemousse.mechanisms.Catapult;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.IMU;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.Static;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Rotation;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.entities.Robot;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector.VectorR;

/**
 * Created by Matthew on 11/14/2016.
 */
@TeleOp(name = "Pamplemousse Drive")
public class FinalTeleOp extends OpMode{

    //initializes motors in order of standard graph quadrants
    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor intakeMotor;

    //Servos used
    private Servo bumperLeft;
    private Servo bumperRight;
    private Servo ballDropper;

    private double intakePower = 0.0;
    private Config rawSettings;
    private Config.ParsedData settings;
    private Config.ParsedData bumpers;
    private Components components;
    private Controllers controls;

    private IMU imuHandler;
    private IMU.Gyrometer gyrometer;
    private BNO055IMU imu;

    private Catapult catapult;
    private Drive drive;
    private VectorR driveVector = new VectorR();
    private Robot robot;
    //private IMU.Gyrometer gyrometer;
    //private IMU imuHandler;

    private static final String TAG = "PamplemousseDebug";

    //Float type variables
    private float bumperPos;
    private float initialX = 0.0f;
    private float initialY = 0.0f;
    private float bumperRange = 0f;
    private float bumperPreset = 0f;
    private float bumperVel = 0f;
    private float bumperMax = 0f;
    private float power = 0f;
    private float imu_offset = 0f;
    private final float twopi = (float) (2.0 * Math.PI);

    //Flags
    private boolean blueTeam;

    //Time archive variable
    private long lastTime = 0;
    private float initialRot;

    public void init() {

        Log.d(TAG, "startingInitNow");

        Config keymapping = new Config(Static.configPath, Static.configControlFileName + Static.configFileSufffix, telemetry, "keymapping");

        Log.d(TAG, "keymapping");

        // should catch other errors, but oh well
        // add data logging
        if (keymapping.read() == Config.State.DEFAULT_EXISTS) {
            keymapping.create(true);
            Log.d(TAG, "keymapping-read-fail-create");
            //read without creating file if it still fails.
            if (keymapping.read() == Config.State.DEFAULT_EXISTS)
                keymapping.read(true);
            Log.d(TAG, "keymapping-read-again");
        }

        Config components = new Config(Static.configPath, Static.configCompFileName + Static.configFileSufffix, telemetry, "components");
        Log.d(TAG, "components");
        if (components.read() == Config.State.DEFAULT_EXISTS) {
            components.create(true);
            Log.d(TAG, "components-read-fail-create");
            if (components.read() == Config.State.DEFAULT_EXISTS)
                components.read(true);
            Log.d(TAG, "components-read-again");
        }

        rawSettings = new Config(Static.configPath, Static.configVarFileName + Static.configFileSufffix, telemetry, "settings");
        Log.d(TAG, "settings");
        if (rawSettings.read() == Config.State.DEFAULT_EXISTS) {
            rawSettings.create(true);
            Log.d(TAG, "settings-read-fail-create");
            if (rawSettings.read() == Config.State.DEFAULT_EXISTS)
                rawSettings.read(true);
            Log.d(TAG, "settings-read-again");
        }

        settings = rawSettings.getParsedData();
        Log.d(TAG, "settings-parse");

        blueTeam = (settings.getString("alliance").equals("blue"));
        initialX = settings.subData("robot").subData("initial_position").getFloat("x");
        initialY = settings.subData("robot").subData("initial_position").getFloat("y");
        initialRot = settings.subData("robot").subData("initial_position").getFloat("initial_rotation");
        robot = new Robot(initialX, initialY, initialRot, blueTeam ? Alliance.BLUE_ALLIANCE : Alliance.RED_ALLIANCE);

        this.components = new Components(hardwareMap, telemetry, components);
        Log.d(TAG, "components-object");
        this.components.initialize();
        Log.d(TAG, "components-init");
        controls = new Controllers(gamepad1, gamepad2, keymapping);
        controls.initialize();
        Log.d(TAG, "controllers-init");

        Log.d(TAG, "STARTING SETUPS");

        //sets variables to motors
        motorFR = hardwareMap.dcMotor.get(settings.subData("drivetrain").subData("motor").subData("front_right").getString("map_name"));
        motorFL = hardwareMap.dcMotor.get(settings.subData("drivetrain").subData("motor").subData("front_left").getString("map_name"));
        motorBL = hardwareMap.dcMotor.get(settings.subData("drivetrain").subData("motor").subData("back_left").getString("map_name"));
        motorBR = hardwareMap.dcMotor.get(settings.subData("drivetrain").subData("motor").subData("back_right").getString("map_name"));

        Log.d(TAG, settings.subData("drivetrain").subData("motor").subData("front_right").getString("map_name"));
        Log.d(TAG, settings.subData("drivetrain").subData("motor").subData("front_left").getString("map_name"));
        Log.d(TAG, settings.subData("drivetrain").subData("motor").subData("back_left").getString("map_name"));
        Log.d(TAG, settings.subData("drivetrain").subData("motor").subData("back_right").getString("map_name"));

        drive = new Drive(driveVector, robot, motorFR, motorFL, motorBL, motorBR, settings, telemetry);

        Log.d(TAG, "DRIVETRAIN SETUP DONE");

        intakeMotor = hardwareMap.dcMotor.get("motorIntake");
        ballDropper = hardwareMap.servo.get("drop");
        if (settings.subData("drop").getBool("reversed")){
            ballDropper.setDirection(Servo.Direction.REVERSE);
        } else{
            ballDropper.setDirection(Servo.Direction.FORWARD);
        }

        Log.d(TAG, "INTAKE AND ARM DONE");

        bumpers = settings.subData("bumper");

        bumperRange = bumpers.getFloat("full_rotate");
        bumperPreset = bumpers.getFloat("preset");
        bumperVel = bumpers.getFloat("max_ang_vel");
        bumperMax = bumpers.getFloat("max_rotate");

        bumperLeft = Aliases.servoMap.get(bumpers.subData("left_servo").getString("map_name"));
        bumperRight = Aliases.servoMap.get(bumpers.subData("right_servo").getString("map_name"));
        if (bumpers.subData("left_servo").getBool("reversed"))
            bumperLeft.setDirection(Servo.Direction.REVERSE);
        if (bumpers.subData("right_servo").getBool("reversed"))
            bumperRight.setDirection(Servo.Direction.REVERSE);

        power = settings.subData("drivetrain").getFloat("motor_scale");

        Log.d(TAG, "BUMPER DONE");

        catapult = new Catapult(hardwareMap.dcMotor.get(settings.subData("catapult").getString("map_name")),
                hardwareMap.opticalDistanceSensor.get(settings.subData("catapult")
                        .subData("light_sensor").getString("map_name")), settings.subData("catapult"));

        Log.d(TAG, "CATAPULT DONE");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imuHandler = new IMU(imu);//todo put in config
        imuHandler.imuInit(); //todo remember to init imu
        gyrometer = imuHandler.getGyrometer();
        gyrometer.enableStream(true);
        imuHandler.setStreamDelay(75);
        imuHandler.streamIMUData();
        robot.getRotation().setRadians(gyrometer.getOrientation(IMU.Axis.Z));
        imuHandler.imuThread.start();

        Log.d(TAG, "IMU SETUP DONE");

        lastTime = System.currentTimeMillis();
        drive.startVelocity();
        drive.setRelative(false);

        Log.d(TAG, "THREAD STARTS DONE");
    }

    public void start(){
        catapult.start();
    }

    public void loop(){
        // TODO: 2/18/2017 move into imu thread
        /*
        if (controls.getDigital("calibrate")){
            imu_offset = twopi + gyrometer.getOrientation(IMU.Axis.Z);
        }
        */
        robot.getRotation().setRadians(twopi + gyrometer.getOrientation(IMU.Axis.Z));
        long changeTime = System.currentTimeMillis() - lastTime;
        lastTime += changeTime;
        if (intakePower < 0){
            intakePower = 0;
        } if (intakePower > 1){
            intakePower = 1;
        }
        intakePower += gamepad1.right_stick_y / settings.getInt("intake_divisor");
        driveVector.setX(controls.getAnalog("drivetrain_x"));
        driveVector.setY(controls.getAnalog("drivetrain_y"));

        driveVector.setRad(controls.getAnalog("drivetrain_rotate"));

        drive.setRelative(controls.getToggle("drive_mode"));

        if (controls.getToggle("intake")){
            intakeMotor.setPower(-Range.scale(intakePower, 0, 1, -.778, .778));
        } else {
            intakeMotor.setPower(0);
        }

        //servo
        dropBalls(controls.getToggle("drop"));

        bumperPos += controls.getAnalog("bumper_angle") * (bumperVel / bumperRange) * ((float) changeTime / 1000.0f);
        if (bumperPos > bumperMax / bumperRange){
            bumperPos = bumperMax / bumperRange;
        }
        if (bumperPos < 0) {
            bumperPos = 0;
        }
        if (controls.getDigital("bumper_preset"))
            bumperPos = bumperPreset / bumperRange;
        bumperLeft.setPosition(bumperPos + bumpers.subData("left_servo").getFloat("offset") / bumperRange);
        bumperRight.setPosition(bumperPos + bumpers.subData("right_servo").getFloat("offset") / bumperRange);

        if (controls.getDigital("launch"))
            catapult.launch();

        telemetry.addData("Speed-FR", motorFR.getPower());
        telemetry.addData("Speed-FL", motorFL.getPower());
        telemetry.addData("Speed-BL", motorBL.getPower());
        telemetry.addData("Speed-BR", motorBR.getPower());
        telemetry.addData("Encoder-FR", motorFR.getCurrentPosition());
        telemetry.addData("Encoder-FL", motorFL.getCurrentPosition());
        telemetry.addData("Encoder-BL", motorBL.getCurrentPosition());
        telemetry.addData("Encoder-BR", motorBR.getCurrentPosition());
        telemetry.addData("x", controls.getAnalog("drivetrain_x"));
        telemetry.addData("y", controls.getAnalog("drivetrain_y"));
        telemetry.addData("rot", controls.getAnalog("drivetrain_rotate"));
        telemetry.addData("IntakeSpeed", intakePower);
        telemetry.addData("IMU drive", controls.getToggle("imu"));
        telemetry.addData("IMU offset", imu_offset);
        telemetry.update();
    }

    @Override
    public void stop() {
        catapult.stop();
        drive.stop();
        imuHandler.imuThread.interrupt();
        imu.close();
    }

    /**
     * Lifts plow if it is down
     * Drops plow if it is lifted
     */
    public void dropBalls(boolean dropBalls){
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
}