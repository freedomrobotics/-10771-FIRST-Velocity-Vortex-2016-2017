package org.fhs.robotics.ftcteam10771.lepamplemousse.modes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.fhs.robotics.ftcteam10771.lepamplemousse.core.ControllersInit;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.StartValues;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.components.Aliases;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.components.Core;

/**
 * Driver controlled class
 */
public class Controlled {
    ControllersInit controls;
    StartValues values = null;
    Telemetry telemetry;
    private long lastTime;      // The time at the last time check (using System.currentTimeMillis())

    //TODO: 12/24/2015 Organize these variables
    float servo_pos = 0.0f;
    boolean servosOff = false;
    boolean plowButton = false;
    long changeTime = 0;
    boolean plowUp = false;

    float armVert_pos = 0.0f;
    float armSweep_pos = 0.0f;
    boolean arm_enabled = false;
    boolean armPressedFlag = false;

    boolean all_clear = false;
    boolean allClearPressedFlag = false;


    /**
     * The constructor for the driver controlled class
     *
     * @param controls    Reference to the object of an initialized ControllersInit class
     * @param startValues Reference to the object of an initialized StartValues class
     * @param telemetry   Reference to the Telemetry object of the OpMode
     */
    public Controlled(ControllersInit controls, StartValues startValues, Telemetry telemetry) {
        this.controls = controls;
        this.values = startValues;
        this.telemetry = telemetry;
        lastTime = System.currentTimeMillis();
        servoSetup();
        //If this is not run, then the plow doesn't get its offset set.
        //togglePlow();
    }

    /**
     * The loop of the controlled class. Does not contain a loop, since it's expected to be within a loop.
     * It lets the drivers drive.
     */
    public void loop() {
        //for any physics, we have the change in time in milliseconds given by changeTime
        changeTime = System.currentTimeMillis() - lastTime;
        lastTime += changeTime;

        //run the drive function
        drive();

        press();

        //functions for the servos
        if (controls.getDigital("servos_off") && !servosOff){
            disableServos();
            servosOff = true;
        }else if (!controls.getDigital("servos_off")){

           // if (controls.getDigital("plow") && !plowButton){
           //     plowUp = !plowUp;
           //     plowButton = true;
           //     //lift or drop the plow
           //     togglePlow();
           // }
           // else if (!controls.getDigital("plow")) {
           //     plowButton = false;
           // }

            //adjusts winch angle
            winchAngle();
            servosOff = false;

            //Control that arm!
            robotArm();

            //Set that All clear!
            allClear();
        }

        if (values.settings("trigger_arm").getBool("enabled")) moveArmTrigger();

        //adjust the length of extension of winch
        extendWinch();

        //Telemetry data for the drivers.
        telemetryData();
    }

    private void allClear() {
        StartValues.Settings a = values.settings("all_clear");
        float fullRange = a.getFloat("full_rotate");
        float offset = a.getFloat("offset") / fullRange;
        float up = a.getFloat("up_angle") / fullRange;
        float down = a.getFloat("down_angle") / fullRange;
        if (all_clear){
            Aliases.servoMap.get("allclear").setPosition(down + offset);
        }else {
            Aliases.servoMap.get("allclear").setPosition(up + offset);
        }
    }

    private void robotArm() {
        if (!arm_enabled)
            return;

        StartValues.Settings sweep = values.settings("robot_arm").getSettings("sweep_servo");
        StartValues.Settings vert = values.settings("robot_arm").getSettings("vert_servo");

        float range = sweep.getFloat("full_rotate");
        armSweep_pos += controls.getAnalog("arm_sweep") * (sweep.getFloat("max_ang_vel") / range) * ((float) changeTime / 1000.0f);

        if (armSweep_pos > sweep.getFloat("max_rotate") / range){
            armSweep_pos = sweep.getFloat("max_rotate") / range;
        }
        if (armSweep_pos < sweep.getFloat("min_rotate") / range) {
            armSweep_pos = sweep.getFloat("min_rotate") / range;
        }
        range = vert.getFloat("full_rotate");
        armVert_pos += controls.getAnalog("arm_vert") * (vert.getFloat("max_ang_vel") / range) * ((float) changeTime / 1000.0f);

        if (armVert_pos > vert.getFloat("max_rotate") / range){
            armVert_pos = vert.getFloat("max_rotate") / range;
        }
        if (armVert_pos < vert.getFloat("min_rotate") / range) {
            armVert_pos = vert.getFloat("min_rotate") / range;
        }

        Aliases.servoMap.get("arm_up").setPosition(armVert_pos);
        Aliases.servoMap.get("arm_side").setPosition(armSweep_pos);
    }

    private void telemetryData() {
        telemetry.addData("Winch Angle", servo_pos * values.settings("winch").getSettings("angular_movement").getFloat("full_rotate"));

        //debug
        telemetry.addData("Arm vert", armVert_pos * values.settings("robot_arm").getSettings("sweep_servo").getFloat("full_rotate"));
        telemetry.addData("Arm sweep", armVert_pos * values.settings("robot_arm").getSettings("vert_servo").getFloat("full_rotate"));
        telemetry.addData("Arm", arm_enabled);

        telemetry.addData("All Clear", all_clear);
    }

    // TODO: 12/24/2015 figure out how to use acutual map_name
    // TODO: 12/20/2015 implement the max_power and map_name
    // TODO: 12/20/2015 To make easier on eyes, put recalled values into a variable or something. The JVM would automatically optimize it.
    /**
     * The function to drive the drivetrain.
     */
    public void drive() {
        float offset = Math.abs(values.settings("drivetrain").getSettings("motor_left").getFloat("offset") - values.settings("drivetrain").getSettings("motor_right").getFloat("offset"));
        if (values.settings("drivetrain").getSettings("motor_left").getFloat("offset") >= values.settings("drivetrain").getSettings("motor_right").getFloat("offset")) {
            offset = StrictMath.copySign(offset, controls.getAnalog("drivetrain_right"));
            if (values.settings("drivetrain").getSettings("motor_left").getBool("reversed")) {
                Aliases.motorMap.get("drive_left").setPower(-controls.getAnalog("drivetrain_left"));
            } else {
                Aliases.motorMap.get("drive_left").setPower(controls.getAnalog("drivetrain_left"));
            }
            if (values.settings("drivetrain").getSettings("motor_right").getBool("reversed")) {
                Aliases.motorMap.get("drive_right").setPower(-(controls.getAnalog("drivetrain_right") - offset));
            } else {
                Aliases.motorMap.get("drive_right").setPower(controls.getAnalog("drivetrain_right") - offset);
            }
        }
        if (values.settings("drivetrain").getSettings("motor_left").getFloat("offset") < values.settings("drivetrain").getSettings("motor_right").getFloat("offset")) {
            offset = StrictMath.copySign(offset, controls.getAnalog("drivetrain_left"));
            if (values.settings("drivetrain").getSettings("motor_right").getBool("reversed")) {
                Aliases.motorMap.get("drive_right").setPower(-controls.getAnalog("drivetrain_right"));
            } else {
                Aliases.motorMap.get("drive_right").setPower(controls.getAnalog("drivetrain_right"));
            }
            if (values.settings("drivetrain").getSettings("motor_left").getBool("reversed")) {
                Aliases.motorMap.get("drive_left").setPower(-(controls.getAnalog("drivetrain_left") - offset));
            } else {
                Aliases.motorMap.get("drive_left").setPower(controls.getAnalog("drivetrain_left") - offset);
            }
        }
    }

    public void press(){
        if (controls.getDigital("arm_toggle") && !armPressedFlag){
            arm_enabled = !arm_enabled;
            armPressedFlag = true;
        } if(!controls.getDigital("arm_toggle")){
            armPressedFlag = false;
        }
        if (controls.getDigital("arm_toggle_on")){
            arm_enabled = true;
        }
        if (controls.getDigital("arm_toggle_off")){
            arm_enabled = false;
        }

        if (controls.getDigital("all_clear") && !allClearPressedFlag){
            all_clear = !all_clear;
            allClearPressedFlag = true;
        } if(!controls.getDigital("all_clear")){
            allClearPressedFlag = false;
        }
        if (controls.getDigital("all_clear_down")){
            all_clear = true;
        }
        if (controls.getDigital("all_clear_up")){
            all_clear = false;
        }
    }

    /**
     * Adjusts the winch's
     * projection angle
     */
    public void winchAngle(){
        if (arm_enabled)
            return;
        StartValues.Settings winch = values.settings("winch");
        StartValues.Settings angular = winch.getSettings("angular_movement");
        float range = angular.getFloat("full_rotate");
        servo_pos += controls.getAnalog("winch_angle") * (angular.getFloat("max_ang_vel") / range) * ((float) changeTime / 1000.0f);

        if (servo_pos > angular.getFloat("max_rotate") / range){
            servo_pos = angular.getFloat("max_rotate") / range;
        }
        if (servo_pos < 0) {
            servo_pos = 0;
        }
        if (controls.getDigital("winch_preset"))
            servo_pos = angular.getFloat("preset") / range;
        Aliases.servoMap.get("winch_left").setPosition(servo_pos + winch.getSettings("left_servo").getFloat("offset") / range);
        Aliases.servoMap.get("winch_right").setPosition(servo_pos + winch.getSettings("right_servo").getFloat("offset") / range);
    }

    /**
     * Switches the servos off
     */
    public void disableServos(){
        for (int i = 0; i < Core.servo.length; i++){
            Core.servo[i].getController().pwmDisable();
        }
    }

    /**
     * Moves the arm trigger of the robot
     */
    public void moveArmTrigger(){
        //Aliases.servoMap.get("arm_trigger").setPosition(controls.getAnalog("trigger_arm"));
        Aliases.motorMap.get("trigger_arm").setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (values.settings("trigger_arm").getString("side").equalsIgnoreCase("left")) Aliases.motorMap.get("trigger_arm").setDirection(DcMotor.Direction.REVERSE);
        else Aliases.motorMap.get("trigger_arm").setDirection(DcMotor.Direction.FORWARD);
        Aliases.motorMap.get("trigger_arm").setPower(values.settings("trigger_arm").getFloat("motor_pow"));
        Aliases.motorMap.get("trigger_arm").setTargetPosition((int)((values.settings("encoder").getInt("output_pulses") / 360) * controls.getAnalog("trigger_arm") * values.settings("trigger_arm").getFloat("motor_range")));
    }

    /**
     * Extends the tape measure
     * of the winch ONLY IF the
     * plow is lifted
     */
    // TODO: 12/28/2015 implement variables from settings.yml
    public void extendWinch(){
        if (arm_enabled)
            return;
        //if (plowUp) {
            Aliases.motorMap.get("winch_motor").setPower(controls.getAnalog("winch_extend_retract"));
        //} else{
        //    Aliases.motorMap.get("winch_motor").setPower(0);
        //}
    }

    /**
     * Lifts plow if it is down
     * Drops plow if it is lifted
     */
    public void togglePlow(){
        StartValues.Settings plow = values.settings("plow");
        float fullRange = plow.getFloat("full_rotate");
        float offset = plow.getFloat("offset") / fullRange;
        float up = plow.getFloat("up_angle") / fullRange;
        float down = plow.getFloat("down_angle") / fullRange;
        if (plowUp){
            Aliases.servoMap.get("plow").setPosition(up + offset);
        }else {
            Aliases.servoMap.get("plow").setPosition(down + offset);
        }
    }

    /**
     * Sets up the servos
     */
    private void servoSetup() {
        if (values.settings("winch").getSettings("left_servo").getBool("reversed")){
            Aliases.servoMap.get("winch_left").setDirection(Servo.Direction.REVERSE);
        }else{
            Aliases.servoMap.get("winch_left").setDirection(Servo.Direction.FORWARD);
        }
        if (values.settings("winch").getSettings("right_servo").getBool("reversed")){
            Aliases.servoMap.get("winch_right").setDirection(Servo.Direction.REVERSE);
        } else {
            Aliases.servoMap.get("winch_right").setDirection(Servo.Direction.FORWARD);
        }
        if (values.settings("trigger_arm").getString("side").equals("right")){
            //Aliases.servoMap.get("arm_trigger").setDirection(Servo.Direction.REVERSE);
        }else{
            //Aliases.servoMap.get("arm_trigger").setDirection(Servo.Direction.FORWARD);
        }
        //if (values.settings("plow").getBool("reversed")){
        //    Aliases.servoMap.get("plow").setDirection(Servo.Direction.REVERSE);
        //}else{
        //    Aliases.servoMap.get("plow").setDirection(Servo.Direction.FORWARD);
        //}


        StartValues.Settings winch = values.settings("winch");
        StartValues.Settings angular = winch.getSettings("angular_movement");
        float range = angular.getFloat("full_rotate");
        servo_pos = angular.getFloat("start_pos") / range;
        Aliases.servoMap.get("winch_left").setPosition(servo_pos + winch.getSettings("left_servo").getFloat("offset") / range);
        Aliases.servoMap.get("winch_right").setPosition(servo_pos + winch.getSettings("right_servo").getFloat("offset") / range);



        if (values.settings("robot_arm").getSettings("sweep_servo").getBool("reversed")){
            Aliases.servoMap.get("arm_side").setDirection(Servo.Direction.REVERSE);
        } else {
            Aliases.servoMap.get("arm_side").setDirection(Servo.Direction.FORWARD);
        }
        if (values.settings("robot_arm").getSettings("vert_servo").getBool("reversed")){
            Aliases.servoMap.get("arm_up").setDirection(Servo.Direction.REVERSE);
        } else {
            Aliases.servoMap.get("arm_up").setDirection(Servo.Direction.FORWARD);
        }
        if (values.settings("all_clear").getBool("reversed")){
            Aliases.servoMap.get("arm_up").setDirection(Servo.Direction.REVERSE);
        } else {
            Aliases.servoMap.get("arm_up").setDirection(Servo.Direction.FORWARD);
        }

        StartValues.Settings sweep = values.settings("robot_arm").getSettings("sweep_servo");
        StartValues.Settings vert = values.settings("robot_arm").getSettings("vert_servo");

        range = sweep.getFloat("full_rotate");
        armSweep_pos = sweep.getFloat("start_pos") / range;
        range = vert.getFloat("full_rotate");
        armVert_pos = vert.getFloat("start_pos") / range;

        Aliases.servoMap.get("arm_up").setPosition(armVert_pos);
        Aliases.servoMap.get("arm_side").setPosition(armSweep_pos);
    }

    public void cleanup(){
        //cleanup code
        Aliases.clearAll();
    }
}