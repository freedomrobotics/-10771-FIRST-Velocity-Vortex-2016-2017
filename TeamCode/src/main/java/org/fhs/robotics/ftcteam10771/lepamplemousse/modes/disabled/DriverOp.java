package org.fhs.robotics.ftcteam10771.lepamplemousse.modes.disabled;

import android.util.Log;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.ApproachBeacon;
import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.Drive;
import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Components;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Controllers;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.components.Aliases;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.IMU;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.Static;
import org.fhs.robotics.ftcteam10771.lepamplemousse.mechanisms.Catapult;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Coordinate;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Rotation;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.entities.Robot;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector.VectorR;

import java.util.LinkedList;
import java.util.List;

import static org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.IMU.Axis.Z;

/**
 * Created by joelv on 2/17/2017.
 */
@Disabled
@TeleOp(name="Driver Op")
public class DriverOp extends LinearOpMode {

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
    private boolean intakeF = false, intakeB = false;
    private List<String> toggle = new LinkedList<>();
    private Config rawSettings;
    private Config.ParsedData settings;
    private Config.ParsedData bumpers;
    //private Components components;
    private Controllers controls;

    private IMU imuHandler;
    private IMU.Gyrometer gyrometer;
    private BNO055IMU imu;

    private Catapult catapult;
    private Drive drive;
    private VectorR driveVector = new VectorR(new Coordinate(), new Rotation());
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

    //Flags
    //private boolean armToggle = false;
    private boolean dropBalls = false;
    private boolean imuOn = false;

    //Time archive variable
    private long lastTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        startOpMode();
        while (opModeIsActive()){
            loopOpMode();
        }
        stopOpMode();
    }

    public void initialize(){
        ApproachBeacon sadkjfhsadk = new ApproachBeacon(settings, drive, this);
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
        /*
        Config components = new Config(Static.configPath, Static.configCompFileName + Static.configFileSufffix, telemetry, "components");
        Log.d(TAG, "components");
        if (components.read() == Config.State.DEFAULT_EXISTS) {
            components.create(true);
            Log.d(TAG, "components-read-fail-create");
            if (components.read() == Config.State.DEFAULT_EXISTS)
                components.read(true);
            Log.d(TAG, "components-read-again");
        }
        */

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

        //blueTeam = (settings.getString("alliance").equals("blue"));
        initialX = settings.subData("robot").subData("initial_position").getFloat("x");
        initialY = settings.subData("robot").subData("initial_position").getFloat("y");

        /*
        this.components = new Components(hardwareMap, telemetry, components);
        Log.d(TAG, "components-object");
        this.components.initialize();
        Log.d(TAG, "components-init");
        */
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

        drive = new Drive(driveVector, new Robot(), motorFR, motorFL, motorBL, motorBR, settings, telemetry);
        drive.setRelative(true);

        Log.d(TAG, "DRIVETRAIN SETUP DONE");

        intakeMotor = hardwareMap.dcMotor.get("motorIntake");
        ballDropper = hardwareMap.servo.get("drop");
        if (settings.subData("drop").getBool("reversed")){
            ballDropper.setDirection(Servo.Direction.REVERSE);
        } else{
            ballDropper.setDirection(Servo.Direction.FORWARD);
        }

        Log.d(TAG, "INTAKE AND ARM DONE");

        /*
        In our mecanum setup, the two front wheels are chain driven and the two rear wheels are
         direct gear driven. What this means is that the two front wheels are spinning the same
         direction as the motor and the two rear wheels are spinning the opposite. Since the
         motors on the right need to be spinning clockwise to move forward, the motors on the
         right need to be reversed. Thus the front right and the back left motors need to be
         reversed. That is, motors A and C. - Adam Li
         */
        /*motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);*/

        bumpers = settings.subData("bumper");

        bumperRange = bumpers.getFloat("full_rotate");
        bumperPreset = bumpers.getFloat("preset");
        bumperVel = bumpers.getFloat("max_ang_vel");
        bumperMax = bumpers.getFloat("max_rotate");

        bumperLeft = hardwareMap.servo.get(bumpers.subData("left_servo").getString("map_name"));
        bumperRight = hardwareMap.servo.get(bumpers.subData("right_servo").getString("map_name"));
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

        Log.d(TAG, "IMU SETUP DONE");

        lastTime = System.currentTimeMillis();
        drive.startVelocity();
        //fixme if catapult crashes the app, comment this and uncomment the other one in startOpMode()
        catapult.start();
        Log.d(TAG, "THREAD STARTS DONE");
    }

    public void startOpMode(){
        //catapult.start();
        return;
    }

    public void loopOpMode(){
        long changeTime = System.currentTimeMillis() - lastTime;
        lastTime += changeTime;
        if (intakePower < 0){
            intakePower = 0;
        } if (intakePower > 1){
            intakePower = 1;
        }
        intakePower += gamepad1.right_stick_y / settings.getInt("intake_divisor");

        imuOn = controls.getToggle("imu");

        double joystickTheta = Math.atan2((controls.getAnalog("drivetrain_y")),(controls.getAnalog("drivetrain_x"))); //declares the angle of joystick position in standard polar coordinates
        //todo see if toggle imuworks

        if (imuOn){
            imuHandler.streamIMUData();
            joystickTheta -= (Math.PI * 2.0) + gyrometer.getOrientation(Z);
        }
        while (joystickTheta<0){
            joystickTheta += (Math.PI * 2.0);
        }
        if (joystickTheta > (2.0 * Math.PI)){
            joystickTheta = joystickTheta % (Math.PI * 2.0);
        }


        double joystickRadius = Math.sqrt((controls.getAnalog("drivetrain_x"))*(controls.getAnalog("drivetrain_x"))+
                (controls.getAnalog("drivetrain_y"))*(controls.getAnalog("drivetrain_y"))); //declares the magnitude of the radius of the joystick position
        // Halved rotationPower value to allow for simultaneous translation and rotation when fully depressed - Adam Li
        double rotationalPower = controls.getAnalog("drivetrain_rotate"); //sets the power of rotation by finding the difference between the left and right triggers
        driveVector.setPolar((float)joystickRadius, (float)joystickTheta);
        driveVector.setRad((float)rotationalPower);

        //sets the motor power where the ratio of input from translational motion is dictated by the magnitude of the rotational motion
            /*
            We know the right side needs to be driven in reverse to rotate right, so motors A
             and D have negative rotationalPower values. We also know forward translation
             involves positive motor values for all when rotation is disregarded, so motors C
             and D have signage on shaft power changed to positive again.
             */
        //todo change these to getToggle()
        /*
        if (gamepad1.a && !toggle.contains("intakeF")){
            intakeF = !intakeF;
            intakeB = false;
            toggle.add("intakeF");
        } if (!gamepad1.a && toggle.contains("intakeF")){
            toggle.remove("intakeF");
        }*/
        intakeF = controls.getToggle("intakeF");

        /*
        if (controls.getDigital("drop") && !toggle.contains("drop")){
            dropBalls = !dropBalls;
            toggle.add("drop");
        } if (!controls.getDigital("drop") && toggle.contains("drop")){
            toggle.remove("drop");
        }*/
        dropBalls = controls.getToggle("drop");

        if (intakeF){
            intakeMotor.setPower(-Range.scale(intakePower, 0, 1, -.778, .778));
        } else {
            intakeMotor.setPower(0);
        }

        //servo
        dropBalls();

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

        double orientation = 0.1;
        if (imuOn) {
            final double full = 2.0 * Math.PI;
            orientation = full + gyrometer.getOrientation(Z);
        }
        telemetry.addData("IMU", orientation);
        telemetry.addData("ImuStatus", imuHandler.isImuInit());
        telemetry.addData("IMUToggle", imuOn);
        telemetry.addData("Radius", driveVector.getRadius());
        telemetry.addData("Theta", driveVector.getTheta());
        telemetry.addData("IntakeSpeed", intakePower);
        telemetry.update();
    }

    public void stopOpMode(){
        catapult.stop();
        drive.stop();
        imuHandler.close();
    }

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
}
