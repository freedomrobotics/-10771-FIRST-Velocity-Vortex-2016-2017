package org.fhs.robotics.ftcteam10771.lepamplemousse.modes.tests;

import android.util.Log;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.Drive;
import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Alliance;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Components;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Controllers;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.IMU;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.RGB;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.UltrasonicRange;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.CameraVision;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.Static;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Coordinate;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Rotation;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.entities.Robot;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector.VectorR;

import static org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.IMU.Axis.Z;

/**
 * Created by joelv on 2/15/2017.
 */
@TeleOp(name="Sensor Testing")
public class SensorTest extends LinearOpMode{

    private Config rawSettings;
    private Config.ParsedData settings;
    private Components components;
    private Controllers controls;
    private static final String TAG = "TestDrive3Debug";

    private OpticalDistanceSensor ods;
    private BNO055IMU imu;
    private IMU imuHandler;
    private IMU.Gyrometer gyrometer;
    private RGB rgb;
    private UltrasonicRange ultraLeft;
    private UltrasonicRange ultraRight;
    private UltrasonicRange ultraBack;
    private Alliance alliance = Alliance.UNKNOWN;

    private Drive drive;
    private VectorR driveVector = new VectorR(new Coordinate(), new Rotation());
    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private boolean led = false;

    @Override
    public void runOpMode() throws InterruptedException {

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

        //todo see if this works for the driver
        this.components = new Components(hardwareMap, telemetry, components);
        Log.d(TAG, "components-object");
        this.components.initialize();
        Log.d(TAG, "components-init");
        controls = new Controllers(gamepad1, gamepad2, keymapping);
        controls.initialize();
        Log.d(TAG, "controllers-init");

        motorFR = hardwareMap.dcMotor.get(settings.subData("drivetrain").subData("motor").subData("front_right").getString("map_name"));
        motorFL = hardwareMap.dcMotor.get(settings.subData("drivetrain").subData("motor").subData("front_left").getString("map_name"));
        motorBL = hardwareMap.dcMotor.get(settings.subData("drivetrain").subData("motor").subData("back_left").getString("map_name"));
        motorBR = hardwareMap.dcMotor.get(settings.subData("drivetrain").subData("motor").subData("back_right").getString("map_name"));

        drive = new Drive(driveVector, new Robot(), motorFR, motorFL, motorBL, motorBR, settings, telemetry);
        ods = hardwareMap.opticalDistanceSensor.get("ods");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        rgb = new RGB(hardwareMap.colorSensor.get("left_rgb"),
                hardwareMap.led.get("left_led"), settings);
        ultraLeft = new UltrasonicRange(hardwareMap.analogInput.get("ultrasonic_left"), hardwareMap.digitalChannel.get("switch_left"));
        ultraRight = new UltrasonicRange(hardwareMap.analogInput.get("ultrasonic_right"), hardwareMap.digitalChannel.get("switch_right"));
        ultraBack = new UltrasonicRange(hardwareMap.analogInput.get("ultrasonic_back"), hardwareMap.digitalChannel.get("switch_back"));
        imuHandler = new IMU(imu);
        gyrometer = imuHandler.getGyrometer();
        gyrometer.enableStream(true);

        if (settings.getString("alliance").equals("red")){
            alliance = Alliance.RED_ALLIANCE;
        }
        else if (settings.getString("alliance").equals("blue")){
            alliance = Alliance.BLUE_ALLIANCE;
        }
        waitForStart();
        rgb.switchLED(false);
        ultraLeft.enable();
        ultraRight.enable();
        ultraBack.enable();
        imuHandler.imuInit();
        drive.startVelocity();
        try {
            while (opModeIsActive()){

                led = controls.getToggle("led");
                rgb.switchLED(led);

                ultraLeft.streamDistance();
                ultraRight.streamDistance();
                ultraBack.streamDistance();
                imuHandler.streamIMUData();

                double joystickTheta = Math.atan2((controls.getAnalog("drivetrain_y")),(controls.getAnalog("drivetrain_x"))); //declares the angle of joystick position in standard polar coordinates
                joystickTheta -= (Math.PI * 2.0) + gyrometer.getOrientation(IMU.Axis.Z);

                //todo see if this works for the driver
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


                telemetry.addData("Alliance", settings.getString("alliance"));
                telemetry.addData("===========", "==============");
                telemetry.addData("UltrasonicL", ultraLeft.getDistance());
                telemetry.addData("UltrasonicR", ultraRight.getDistance());
                telemetry.addData("UltrasonicB", ultraBack.getDistance());
                telemetry.addData("===========", "==============");
                telemetry.addData("Hue-L", rgb.getHue(RGB.Direction.LEFT));
                telemetry.addData("Saturation-L", rgb.getSaturation(RGB.Direction.LEFT));
                telemetry.addData("Brightness-L", rgb.getBrightness(RGB.Direction.LEFT));
                telemetry.addData("Blue-L", rgb.blue());
                telemetry.addData("Red", rgb.red());
                telemetry.addData("Green", rgb.green());
                telemetry.addData("===========", "==============");
                telemetry.addData("Gyro-Z", Math.abs(gyrometer.convert(Z, gyrometer.getOrientation(Z))));
                telemetry.addData("ODS", ods.getLightDetected());
                telemetry.update();
            }
        } catch (Exception e){
            drive.stop();
        }

    }
}
