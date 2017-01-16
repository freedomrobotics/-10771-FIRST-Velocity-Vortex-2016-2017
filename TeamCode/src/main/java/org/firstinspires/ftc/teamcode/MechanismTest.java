package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;

/**
 * Tests the intake and launch mechanism
 * Created by joelv on 1/16/2017.
 */
@TeleOp(name="LauncherTest")
public class MechanismTest extends LinearOpMode{

    DcMotor intakeMotor;
    DcMotor launchMotor;
    Launcher launcher;

    @Override
    public void runOpMode() throws InterruptedException {
        intakeMotor = hardwareMap.dcMotor.get("intake");
        launchMotor = hardwareMap.dcMotor.get("launcher");
        launcher = new Launcher(intakeMotor, DcMotorSimple.Direction.FORWARD, launchMotor, DcMotorSimple.Direction.FORWARD);
        waitForStart();
        while(opModeIsActive()){
            launcher.adjustPower(gamepad1.right_stick_y);
            launcher.increasePower(gamepad1.y);
            launcher.decreasePower(gamepad1.a);
            launcher.launch(gamepad1.right_bumper);
            launcher.intake(gamepad1.left_bumper);
            telemetry.addData("Launch Motor Power", launcher.getLaunchPower());
            telemetry.update();
        }
    }
}
