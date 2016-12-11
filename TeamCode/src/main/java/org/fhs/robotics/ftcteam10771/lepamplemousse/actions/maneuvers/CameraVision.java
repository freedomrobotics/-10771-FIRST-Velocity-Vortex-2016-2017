package org.fhs.robotics.ftcteam10771.lepamplemousse.actions.maneuvers;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera.Camera;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

/**
 * Created by Freedom Robotics on 11/12/2016.
 */

public class CameraVision {

    public CameraVision(){

    }

    public boolean vuforiaRunning = false;
    double degreesToTurn = 0;
    VuforiaLocalizer.Parameters params = null;
    VuforiaLocalizer vuforia = null;
    VuforiaTrackables beacons = null;
    ImageData[] imageData = null;

    public class ImageData{
        OpenGLMatrix matrix;
        OpenGLMatrix rotatedMatrix;
        VectorF translation;
    }

    public void vuforiaInit() {
        params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "AVj2kiX/////AAAAGV0J5W5oOkUTvP1+IKxrWdIpD63oQV8zSY/+qSNDxkt5zj8tW0N9AK7/3yUJRBlnJx80gStuZcHF7JoiKUNj4JmO6gcyIQn2LWZ/0hL9gFM+PmwM6lvzJu9U/gmvf++GngzR74ft0gjlNPle9qDHEaAgMHYcbEDpc4msHDVn6ZjCcxDem2tQyW4gEY334fwAU9E0ySkw1KwC/Mo6gaE7bW1Mh9xLbXYTe2+sRclEA6YbrKeH8LHmJBDXQxTdcL4HyS26oPYAGRXfFLoi7QkBdkPDYKiPQUsCoHhNz1uhPh5duEdwOD9Sm6YUPZYet7Mo9QP3sxaDlaqY5l2pHYn/tH31Xu9eqLKe2RmNRzgMNaJ9\n";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
        beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");
        imageData = new ImageData[beacons.size()];
        for (int i=0; i < beacons.size(); i++){
            imageData[i] = new ImageData();
        }
        beacons.activate();
        vuforiaRunning = true;
    }

    public void runImageTracking(VisionTargetTracker opMode) {
        for (int i=0; i < beacons.size(); i++) {
            imageData[i].matrix = ((VuforiaTrackableDefaultListener) beacons.get(i).getListener()).getPose();
            if (imageData[i].matrix != null) {
                imageData[i].translation = imageData[i].matrix.getTranslation();
                opMode.telemetry.addData(beacons.get(i).getName() + "-translation", imageData[i].translation);
                opMode.telemetry.addData(beacons.get(i).getName() + "-matrix", imageData[i].matrix);
                opMode.telemetry.addData(beacons.get(i).getName() + "-perpendicularness", imageData[i].matrix.getData()[0]);
                opMode.telemetry.addData(beacons.get(i).getName() + "-degreesToturn", imageData[i].matrix.getData()[8]);
                //translation.get(0) returns x-component of vector: positive numbers towards left
                //translation.get(1) returns y-component of vector: positive numbers downwards
                //translation.get(2) returns z-component of vector: distance = abs(negative number)
            }
            else opMode.telemetry.addData("Null" + i, "null");
        }
        opMode.telemetry.addData("Number of Images", countTrackedImages());
        opMode.telemetry.addData("Iterations", opMode.iteration);
        opMode.telemetry.update();
    }

    public int countTrackedImages(){
        int images = 0;
        for (int i=0; i < beacons.size(); i++){
            if (imageData[i]!=null) {
                if (imageData[i].matrix != null) {
                    images++;
                }
            }
        }
        return images;
    }

    public String getBeaconName(int index){
        return beacons.get(index).getName();
    }
}
