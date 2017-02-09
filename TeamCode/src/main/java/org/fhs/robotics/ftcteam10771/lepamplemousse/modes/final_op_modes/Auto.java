package org.fhs.robotics.ftcteam10771.lepamplemousse.modes.final_op_modes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Components;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Controllers;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.components.Aliases;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.Static;

import java.util.LinkedList;
import java.util.List;

/**
 * Rough autonomous from framework_test
 * Created by joelv on 2/9/2017.
 */

public class Auto extends LinearOpMode{

    //initializes motors in order of standard graph quadrants
    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private DcMotor intakeMotor;
    private double intakePower = 0.0;
    private boolean intakeF = false, intakeB = false;
    private List<String> toggle = new LinkedList<>();
    private Config rawSettings;
    private Config.ParsedData settings;
    private Components components;
    private Controllers controls;

    private static final String TAG = "TestDrive3Debug";

    @Override
    public void runOpMode() throws InterruptedException{

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


        this.components = new Components(hardwareMap, telemetry, components);
        Log.d(TAG, "components-object");
        this.components.initialize();
        Log.d(TAG, "components-init");
        controls = new Controllers(gamepad1, gamepad2, keymapping);
        controls.initialize();
        Log.d(TAG, "controllers-init");

        //sets variables to motors
        motorFR = hardwareMap.dcMotor.get(settings.subData("drivetrain").subData("motor").subData("front_right").getString("map_name"));
        motorFL = hardwareMap.dcMotor.get(settings.subData("drivetrain").subData("motor").subData("front_left").getString("map_name"));
        motorBL = hardwareMap.dcMotor.get(settings.subData("drivetrain").subData("motor").subData("back_left").getString("map_name"));
        motorBR = hardwareMap.dcMotor.get(settings.subData("drivetrain").subData("motor").subData("back_right").getString("map_name"));

        Log.d(TAG, settings.subData("drivetrain").subData("motor").subData("front_right").getString("map_name"));
        Log.d(TAG, settings.subData("drivetrain").subData("motor").subData("front_left").getString("map_name"));
        Log.d(TAG, settings.subData("drivetrain").subData("motor").subData("back_left").getString("map_name"));
        Log.d(TAG, settings.subData("drivetrain").subData("motor").subData("back_right").getString("map_name"));

        Log.d(TAG, "motor setup");

        intakeMotor = hardwareMap.dcMotor.get("motorIntake");

        Log.d(TAG, "motor setup 2");

        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Log.d(TAG, "motor setup 3");

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

        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        if (settings.subData("drivetrain").subData("motor").subData("front_right").getBool("reversed")) motorFR.setDirection(DcMotor.Direction.REVERSE);
        if (settings.subData("drivetrain").subData("motor").subData("front_left").getBool("reversed")) motorFL.setDirection(DcMotor.Direction.REVERSE);
        if (settings.subData("drivetrain").subData("motor").subData("back_left").getBool("reversed")) motorBL.setDirection(DcMotor.Direction.REVERSE);
        if (settings.subData("drivetrain").subData("motor").subData("back_right").getBool("reversed")) motorBR.setDirection(DcMotor.Direction.REVERSE);

        Log.d(TAG, "motor setup 4");

        waitForStart();

        sleep(settings.subData("auto").getInt("delay"));

        double powerA = settings.subData("auto").getFloat("power");
        double powerB = settings.subData("auto").getFloat("power");
        double powerC = settings.subData("auto").getFloat("power");
        double powerD = settings.subData("auto").getFloat("power");
        motorFR.setPower(powerA);
        motorFL.setPower(powerB);
        motorBL.setPower(powerC);
        motorBR.setPower(powerD);

        waitForNextHardwareCycle();

        sleep(settings.subData("auto").getInt("drive"));

        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

        Aliases.clearAll();
    }
}
