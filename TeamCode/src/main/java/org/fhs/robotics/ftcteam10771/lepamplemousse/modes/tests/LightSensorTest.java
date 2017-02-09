package org.fhs.robotics.ftcteam10771.lepamplemousse.modes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by joelv on 2/8/2017.
 */
@Autonomous (name="Light Sensor Test", group = "Test")
public class LightSensorTest extends LinearOpMode {

    OpticalDistanceSensor sensor;
    DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.opticalDistanceSensor.get("ods");
        motor = hardwareMap.dcMotor.get("launcher");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        while (opModeIsActive()){
            motor.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            telemetry.addData("Light", sensor.getLightDetected());
            telemetry.addData("Raw Light", sensor.getRawLightDetected());
            telemetry.addData("Position", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
