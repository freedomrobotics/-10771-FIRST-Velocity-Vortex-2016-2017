package org.fhs.robotics.ftcteam10771.lepamplemousse.modes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.scriptedconfig.ScriptLoader;
import org.fhs.robotics.ftcteam10771.lepamplemousse.actions.scriptedconfig.ScriptRunner;
import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Components;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Controllers;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.Static;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.entities.Robot;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector.VectorR;

import java.util.List;

/**
 * Created by Freedom Robotics on 2/10/2017.
 */
public class AtomScriptTester extends LinearOpMode implements ScriptRunner {
    Controllers controls;
    private long lastTime;      // The time at the last time check (using System.currentTimeMillis())
    private Config rawSettings;
    private Config.ParsedData settings;
    private Components components;
    private Robot robot;

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

        // TODO: 2/10/2017 Initialize for realz
        robot = new Robot();

        this.components = new Components(hardwareMap, telemetry, components);
        this.components.initialize();
        this.controls = new Controllers(gamepad1, gamepad2, keymapping);
        this.controls.initialize();

        //PREP THE COMMAND LIST!
        if (settings.subData("autonomous").getObject("command_list") == null)
            return;

        List<String> commandList = (List<String>) settings.subData("autonomous").getObject("command_list");

        int counter = 1; //debug

        //WAIT FOR THE STARTI!@HTIOgRFOIWUQ#GQIOH
        waitForStart();
        this.lastTime = System.currentTimeMillis();

        while(opModeIsActive()){
            for(String command : commandList){
                //check if opmode is active
                if (!opModeIsActive()) break;

                ScriptLoader.CommandParser commandParser = new ScriptLoader.CommandParser(command);

                commandPicker(commandParser);

                if (commandParser.command().equalsIgnoreCase("stop")){
                    requestOpModeStop();
                }

                //Debug section
                telemetry.addData("robot", robot.toString());
                telemetry.addData(counter + "", command);
                counter++;
            }
        }
    }

    @Override
    public void commandPicker(ScriptLoader.CommandParser commandParser) throws InterruptedException {

        if (commandParser.command().equalsIgnoreCase("move_position")){
            if (commandParser.getArgsSize() == 2){
                //define function for a 2 argument command
                //drive(commandParser.getArgFloat(0), commandParser.getArgFloat(1));
            } if(commandParser.getArgsSize() >= 3){
                VectorR temp = new VectorR();
                temp.setX(commandParser.getArgFloat(0));
                temp.setY(commandParser.getArgFloat(1));
                temp.setRad(commandParser.getArgFloat(2));
                //move_position 3.5,3,500
            }
            return;

        }

        if (commandParser.command().equalsIgnoreCase("tool")){
            return;
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
}