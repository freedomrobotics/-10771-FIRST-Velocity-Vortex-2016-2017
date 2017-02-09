package org.fhs.robotics.ftcteam10771.lepamplemousse.modes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Controllers;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.Static;

import java.util.List;

/**
 * Created by joelv on 2/7/2017.
 */
@Autonomous(name="Script Test", group = "Test")
public class ScriptTester extends LinearOpMode {

    Config fieldMapConfig;
    Config.ParsedData parsedField;
    Controllers controls;

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

        this.controls = new Controllers(gamepad1, gamepad2, keymapping);
        this.controls.initialize();

        fieldMapConfig = new Config("/Position", "fieldmap.yml", telemetry, "field");
        if (fieldMapConfig.read()== Config.State.DEFAULT_EXISTS){
            fieldMapConfig.create(true);
            if (fieldMapConfig.read()== Config.State.DEFAULT_EXISTS)
                fieldMapConfig.read(true);
        }

        parsedField = fieldMapConfig.getParsedData().subData("coordinates").subData("red");
        waitForStart();
        startScript();



    }


    /**
     * For this format in YML
     *
     * script:
     *  - corner
     *  - center
     *  - stuff
     */
    public void startScript(){
        List<String> commands = (List<String>) parsedField.getObject("script");
        for (String command : commands){
            /*
            while(!controls.getDigital("script")){}
            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            */
            long startTime = System.currentTimeMillis();
            while (System.currentTimeMillis()-startTime<10000){
                telemetry.addData("Location", command);
                telemetry.addData("X", parsedField.subData(command).getFloat("x"));
                telemetry.addData("Y", parsedField.subData(command).getFloat("y"));
                telemetry.update();
            }
        }
    }

    /**
     * Plan C: Do this format in yml
     *
     * command1: center
     * command2: corner
     * command3: stuff
     */
    public void startCommands() {
        int script_size = parsedField.getInt("script_size");
        String location;
        for (int i = 1; i <= script_size; i++) {
            location = parsedField.getString("command" + Integer.toString(i));
            telemetry.addData("X", parsedField.subData(location).getFloat("x"));
            telemetry.addData("Y", parsedField.subData(location).getFloat("y"));
            telemetry.update();
            try {
                wait(10000);
            } catch (Exception e) {
                break;
            }
        }
    }

    /**
     * Plan B: This format
     *
     * command:
     *  1: corner
     *  2: center
     *  3: beacon
     */
    public void startList(){
        int script_size = parsedField.getInt("script_size");
        String location;
        for (int i = 1; i <= script_size; i++) {
            location = parsedField.subData("command").getString(Integer.toString(i));
            telemetry.addData("X", parsedField.subData(location).getFloat("x"));
            telemetry.addData("Y", parsedField.subData(location).getFloat("y"));
            telemetry.update();
            try {
                wait(10000);
            } catch (Exception e) {
                break;
            }
        }
    }
}
