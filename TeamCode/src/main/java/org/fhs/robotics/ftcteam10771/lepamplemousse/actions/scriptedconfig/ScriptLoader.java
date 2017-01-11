package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.scriptedconfig;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Config;

import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Created by Adam Li on 1/11/2017.
 * todo write and fix
 * based off of the previous atomfunctions
 */

public class ScriptLoader {
    Config.ParsedData scriptedAutonomousData;

    ScriptLoader(Config.ParsedData scriptedAutonomousData){
        this.scriptedAutonomousData = scriptedAutonomousData
    }

    public static class CommandParser{
        String command;
        List<Object> arguments = new ArrayList<Object>();

        public CommandParser(String command){
            this.command = command;
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


}

/*
package org.fhs.robotics.ftcteam10771.lepamplemousse.computations;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robocol.Telemetry;

import org.fhs.robotics.ftcteam10771.lepamplemousse.computations.spatialmapping.maps.Maps;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.StartValues;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.components.Aliases;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Created by Adam Li on 2/18/2016.
 *//*
public class AtomFunctionsTake2 {

    Maps fieldMap;
    StartValues values = null;
    float servo_pos;
    LinearOpMode parent = null;
    Telemetry telemetry;

    public AtomFunctionsTake2(StartValues values, Maps fieldMap, LinearOpMode parent, Telemetry telemetry){
        this.values = values;
        this.fieldMap = fieldMap;
        this.parent = parent;
        this.telemetry = telemetry;
    }

    public static class CommandParser{
        String command;
        List<Object> arguments = new ArrayList<Object>();

        public CommandParser(String command){
            this.command = command;
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



    public void drive(float power) throws InterruptedException {
        parent.waitForNextHardwareCycle();
        if (values.settings("drivetrain").getSettings("motor_left").getBool("reversed"))
            Aliases.motorMap.get("drive_left").setPower(-power);
        else
            Aliases.motorMap.get("drive_left").setPower(power);
        if (values.settings("drivetrain").getSettings("motor_right").getBool("reversed"))
            Aliases.motorMap.get("drive_right").setPower(-power);
        else
            Aliases.motorMap.get("drive_right").setPower(power);
        parent.waitOneFullHardwareCycle();
    }


    // TODO: 2/19/2016 Finish and add Map references back in
    // TODO: 2/19/2016 Make sure robot doesn't ram into wall
    public void move(float distance) throws InterruptedException {
        float wheel_dia = values.settings("drivetrain").getSettings("wheel").getFloat("diameter");
        double startPos = getPosition(Aliases.motorMap.get("drive_left"), wheel_dia);
        double endPos = startPos + distance * 10.0f;
        double travelled;
        if (endPos >= startPos)
            drive(0.5f);
        else
            drive(-0.5f);
        while (true){
            if (endPos >= startPos) {
                if ((travelled = getPosition(Aliases.motorMap.get("drive_left"), wheel_dia)) >= endPos) {
                    break;
                }
            }else{
                if ((travelled = getPosition(Aliases.motorMap.get("drive_left"), wheel_dia)) <= endPos) {
                    break;
                }
            }
            telemetry.addData("currentPos", travelled);
            telemetry.addData("goal", endPos);
            parent.waitForNextHardwareCycle();
        }
        drive(0);

        travelled -= startPos;

        travelled /= 1000.0f;

        float x = fieldMap.getRobot().getPosition().getX();
        float y = fieldMap.getRobot().getPosition().getY();
        float rot = fieldMap.getRobot().getRotation().getRadians();

        fieldMap.getRobot().getPosition().setX(x + (float) (Math.cos(rot) * travelled));
        fieldMap.getRobot().getPosition().setY(y + (float) (Math.sin(rot) * travelled));
    }

    public void move(float pointX, float pointY) throws InterruptedException {
        rotate(pointX, pointY);
        float changeX = pointX - fieldMap.getRobot().getPosition().getX();
        float changeY = pointY - fieldMap.getRobot().getPosition().getY();
        float distance = (float)Math.sqrt(changeX*changeX + changeY*changeY);
        move(distance * 100);
    }

    public void rotate(float degrees) throws InterruptedException {
        long time = (long) Math.abs((degrees / (360.0f * 5.0f)) * values.settings("autonomous").getInt("rotation_s"));
        //left
        float power = 1;
        //right
        if (degrees < 0)
            power = -1;

        parent.waitForNextHardwareCycle();
        if (values.settings("drivetrain").getSettings("motor_left").getBool("reversed"))
            Aliases.motorMap.get("drive_left").setPower(power);
        else
            Aliases.motorMap.get("drive_left").setPower(-power);
        if (values.settings("drivetrain").getSettings("motor_right").getBool("reversed"))
            Aliases.motorMap.get("drive_right").setPower(-power);
        else
            Aliases.motorMap.get("drive_right").setPower(power);

        parent.sleep(time);

        drive(0);

        float rot = fieldMap.getRobot().getRotation().getDegrees() + degrees;
        while (rot > 360.0f){
            rot = rot - 360.0f;
        }
        while (rot < 0){
            rot = rot + 360.0f;
        }
        fieldMap.getRobot().getRotation().setDegrees(rot);
    }

    public void rotate(float pointX, float pointY) throws InterruptedException {
        float changeX = pointX - fieldMap.getRobot().getPosition().getX();
        float changeY = pointY - fieldMap.getRobot().getPosition().getY();

        float newRot = (float) Math.atan(changeY/changeX);

        float changeRot = newRot - fieldMap.getRobot().getRotation().getRadians();

        rotate((changeRot / (float)Math.PI) * 180.0f);
    }

    public void winchServo(float degrees) throws InterruptedException {
        if (degrees > values.settings("winch").getSettings("angular_movement").getFloat("max_rotate")){
            degrees = values.settings("winch").getSettings("angular_movement").getFloat("max_rotate");
        }
        if (degrees < 0) {
            degrees = 0;
        }
        float targetServoPos = degrees / values.settings("winch").getSettings("angular_movement").getFloat("full_rotate");
        long lastTime = System.currentTimeMillis();
        while (targetServoPos != servo_pos) {
            //for any physics, we have the change in time in milliseconds given by changeTime
            long changeTime = System.currentTimeMillis() - lastTime;
            lastTime += changeTime;

            if (targetServoPos > servo_pos) {
                servo_pos += (values.settings("winch").getSettings("angular_movement").getFloat("max_ang_vel") / values.settings("winch").getSettings("angular_movement").getFloat("full_rotate")) * ((float) changeTime / 1000.0f);
                if (servo_pos > targetServoPos) {
                    servo_pos = targetServoPos;
                    break;
                }
            }
            if (targetServoPos < servo_pos) {
                servo_pos -= (values.settings("winch").getSettings("angular_movement").getFloat("max_ang_vel") / values.settings("winch").getSettings("angular_movement").getFloat("full_rotate")) * ((float) changeTime / 1000.0f);
                if (servo_pos < targetServoPos) {
                    servo_pos = targetServoPos;
                    break;
                }
            }
            updateWinchServos();
            parent.waitOneFullHardwareCycle();
        }
    }

    public void winchServo(boolean up, float degrees) throws InterruptedException {
        float currentPos = servo_pos * values.settings("winch").getSettings("angular_movement").getFloat("full_rotate");
        if (up){
            winchServo(currentPos * values.settings("winch").getSettings("angular_movement").getFloat("full_rotate") + degrees);
        }else{
            winchServo(currentPos * values.settings("winch").getSettings("angular_movement").getFloat("full_rotate") - degrees);
        }
    }

    public void clearArm(){

    }


    Map<DcMotor, Integer> lastEncoderPulse = new HashMap<DcMotor, Integer>();


    public double getChangeEncoder(DcMotor dcMotor){
        int changeEncoder = 0;
        if (lastEncoderPulse.get(dcMotor) != null) {
            changeEncoder = dcMotor.getCurrentPosition() - lastEncoderPulse.get(dcMotor);
            int last = lastEncoderPulse.get(dcMotor) + changeEncoder;
            lastEncoderPulse.put(dcMotor, last);
        } else {
            lastEncoderPulse.put(dcMotor, dcMotor.getCurrentPosition());
        }
        return changeEncoder;
    }

    /**
     *
     * @param dcMotor   The motor
     * @param diameter  The diameter
     * @return change in distance in millimeters
     *//*
    public double getChangePosition(DcMotor dcMotor, float diameter){
        double changeEncoder = getChangeEncoder(dcMotor);
        int encoderFull = values.settings("encoder").getInt("output_pulses");
        return Math.PI * diameter * (changeEncoder / (float) encoderFull);
    }

    public double getPosition(DcMotor dcMotor, float diameter){
        int pos = dcMotor.getCurrentPosition();
        telemetry.addData("encoder", pos);
        int encoderFull = values.settings("encoder").getInt("output_pulses");
        return Math.PI * diameter * ((float) pos / (float) encoderFull);
    }

    public void setServoPos(float servo_pos){
        this.servo_pos = servo_pos;
    }

    public float getServo_pos(){
        return servo_pos;
    }

    /**
     * Adjusts the winch's
     * projection angle
     *//*
    private void updateWinchServos(){
        StartValues.Settings winch = values.settings("winch");
        StartValues.Settings angular = winch.getSettings("angular_movement");
        float range = angular.getFloat("full_rotate");

        if (servo_pos > angular.getFloat("max_rotate") / range){
            servo_pos = angular.getFloat("max_rotate") / range;
        }
        if (servo_pos < 0) {
            servo_pos = 0;
        }
        Aliases.servoMap.get("winch_left").setPosition(servo_pos + winch.getSettings("left_servo").getFloat("offset") / range);
        Aliases.servoMap.get("winch_right").setPosition(servo_pos + winch.getSettings("right_servo").getFloat("offset") / range);
    }
}
 */