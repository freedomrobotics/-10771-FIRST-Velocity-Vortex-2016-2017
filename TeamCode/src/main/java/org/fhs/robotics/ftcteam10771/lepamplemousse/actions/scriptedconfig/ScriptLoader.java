package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.scriptedconfig;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.Alliance;

import java.util.ArrayList;
import java.util.List;
import java.util.StringTokenizer;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Created by Adam Li on 1/11/2017.
 * todo write and fix
 * based off of the previous atomfunctions
 *
 * In the future, I want to reduce the needs for an entire additional function in the actual opmode to handle the parsed commands
 */

public class ScriptLoader {
    private Config.ParsedData scriptedAutonomousData;
    private List<CommandParser> commandList;

    ScriptLoader(Config.ParsedData scriptedAutonomousData){
        this.scriptedAutonomousData = scriptedAutonomousData;

        //load default config. Dangerous action, but oh well.
        if (!loadScript("defailt"))
            commandList = new ArrayList<>();
    }

    /**
     * Load a specific script
     * @param scriptName
     * @return true if loaded, false if nonexistant
     */
    public boolean loadScript(String scriptName){
        if (!scriptedAutonomousData.valueExists(scriptName)){
            return false;
        }
        List<String> commands = (List<String>) scriptedAutonomousData.getObject(scriptName);
        for (String command : commands) {
            CommandParser parsedCommand = new CommandParser(command);
            commandList.add(parsedCommand);
        }
        return true;
    }

    public static class CommandParser{
        String command, rawCommand;
        List<Object> arguments = new ArrayList<Object>();

        public CommandParser(String command){
            this.command = this.rawCommand = command;
            if (command.contains(" ")) {
                this.command = command.split(" ", 2)[0];
                String argumentList = command.split(" ", 2)[1];
                argumentList = argumentList.replaceAll("\\s", "");
                for (String arg : argumentList.split(",")) {
                    if (arg.length() != 0)
                        arguments.add(arg);
                }
            }
        }

        public String getRawCommand(){
            return rawCommand;
        }

        public String command(){
            return command;
        }

        public int getArgsSize(){
            return arguments.size();
        }

        public Object getArgObject(int loc) {
            if (loc <= arguments.size()) {
                return arguments.get(loc);
            }
            return null;
        }

        public boolean getArgBool(int loc) {
            if (loc <= arguments.size()) {
                if (arguments.get(loc) != null)
                    return arguments.get(loc).toString().equals("true");
            }
            return false;
        }

        public String getArgString(int loc) {
            if (loc <= arguments.size()) {
                if (arguments.get(loc) != null)
                    return arguments.get(loc).toString();
            }
            return null;
        }

        public int getArgInt(int loc) {
            if (loc <= arguments.size()) {
                if (arguments.get(loc) != null) {
                    Pattern p = Pattern.compile("[a-zA-Z]");
                    Matcher m = p.matcher(arguments.get(loc).toString());
                    if (m.find()) {
                        return 0;
                    }
                    if (arguments.get(loc).toString().contains(".")) {
                        return ((Double) Double.parseDouble(arguments.get(loc).toString())).intValue();
                    }
                    return Integer.parseInt(arguments.get(loc).toString());
                }
            }
            return 0;
        }

        public float getArgFloat(int loc) {
            if (loc <= arguments.size()) {
                if (arguments.get(loc) != null) {
                    Pattern p = Pattern.compile("[a-zA-Z]");
                    Matcher m = p.matcher(arguments.get(loc).toString());
                    if (m.find()) {
                        return 0;
                    }
                    if (!arguments.get(loc).toString().contains(".")) {
                        return ((Integer) Integer.parseInt(arguments.get(loc).toString())).floatValue();
                    }
                    return ((Double) Double.parseDouble(arguments.get(loc).toString())).floatValue();
                }
            }
            return 0;
        }
    }

    public List<CommandParser> getCommandList(){
        return commandList;
    }


}