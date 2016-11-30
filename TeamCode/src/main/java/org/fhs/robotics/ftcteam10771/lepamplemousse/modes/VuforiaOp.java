package org.fhs.robotics.ftcteam10771.lepamplemousse.modes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcontroller.internal.FTCVuforia;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.util.HashMap;

/**
 * Created by Shabi on 11/19/2016.
 */

public class VuforiaOp extends OpMode {
    FTCVuforia vuforia;

    @Override
    public void init() {
        vuforia = FtcRobotControllerActivity.getVuforia();
        vuforia.addTrackables("Ball_OT.xml");
        vuforia.initVuforia();
    }
    @Override
    public void loop() {
        HashMap<String, double[]> data = vuforia.getVuforiaData();

        if(data.containsKey("Ball")) {
            telemetry.addData("Ball", data.get("Ball")[0]);


        }

    }

    public void stop() {
        super.stop();
        vuforia.destroy();
    } catch (Exception e) {
        e.printStackTrace();
    }

}
