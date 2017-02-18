package org.fhs.robotics.ftcteam10771.lepamplemousse.modes.disabled_tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Components;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Controllers;
import org.fhs.robotics.ftcteam10771.lepamplemousse.mechanisms.CatapultOld;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.Static;

import java.util.LinkedList;
import java.util.List;

/**
 * Created by joelv on 2/7/2017.
 */
@Disabled
@TeleOp(name="CatapultOld Test", group = "Test")
public class CatapultTest extends OpMode{

    private List<String> toggle = new LinkedList<>();
    private Config rawSettings;
    private Config.ParsedData settings;
    private Components components;
    private Controllers controls;
    private CatapultOld catapult;

    private static final String TAG = "TestDrive3Debug";
    private float bumperPos;
    private float initialX = 0.0f;
    private float initialY = 0.0f;

    @Override
    public void init() {
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

        //FIXME: not sure if this initialization will work
        //TODO: before compiling, enable motor6 and light sensor to true and configurate them or change this line of code
        catapult = new CatapultOld(hardwareMap.dcMotor.get(settings.subData("catapult").getString("map_name")), hardwareMap.opticalDistanceSensor.get("ods"), controls, settings);
    }

    @Override
    public void start() {
        catapult.catapultThread.start();
    }

    @Override
    public void loop(){
        telemetry.addData("Position", catapult.getCatapultPosition());
        telemetry.addData("Readiness", catapult.encoderReady());
        telemetry.addData("Light", catapult.getLight());
        telemetry.addData("Target", catapult.getTargetPosition());
        telemetry.addData("ReadyPosition", catapult.getReadyPosition());
        telemetry.update();
    }


    @Override
    public void stop() {
        catapult.catapultThread.interrupt();
    }
}
