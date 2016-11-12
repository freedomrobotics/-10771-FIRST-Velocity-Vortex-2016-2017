package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

/**
 * Created by Freedom Robotics on 11/12/2016.
 */
@Autonomous(name = "Vuforia", group = "10771")
public class CameraVision extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AVj2kiX/////AAAAGV0J5W5oOkUTvP1+IKxrWdIpD63oQV8zSY/+qSNDxkt5zj8tW0N9AK7/3yUJRBlnJx80gStuZcHF7JoiKUNj4JmO6gcyIQn2LWZ/0hL9gFM+PmwM6lvzJu9U/gmvf++GngzR74ft0gjlNPle9qDHEaAgMHYcbEDpc4msHDVn6ZjCcxDem2tQyW4gEY334fwAU9E0ySkw1KwC/Mo6gaE7bW1Mh9xLbXYTe2+sRclEA6YbrKeH8LHmJBDXQxTdcL4HyS26oPYAGRXfFLoi7QkBdkPDYKiPQUsCoHhNz1uhPh5duEdwOD9Sm6YUPZYet7Mo9QP3sxaDlaqY5l2pHYn/tH31Xu9eqLKe2RmNRzgMNaJ9\n";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        boolean test = Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_OBJECT_TARGETS, 4);
        boolean test2 = Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");

        waitForStart();

        beacons.activate();

        while (opModeIsActive()) {
            for (VuforiaTrackable beac : beacons) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();

                if (pose != null) {
                    VectorF translation = pose.getTranslation();

                    telemetry.addData(beac.getName() + "-Translation", translation);

                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));

                    telemetry.addData(beac.getName() + "-Degrees", degreesToTurn);
                }
                else {
                    telemetry.addData("Objects:", "None");
                }

            }
            telemetry.addData("Hint",test);
            telemetry.addData("Hint2",test2);
            telemetry.update();
        }
    }
}
