package org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera;

import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

/**
 * Class that handles camera vision targeting
 * for the four images under the beacons
 * TODO: Test the class to validate that it works
 *
 * Created by joelv on 1/11/2017.
 */
public class CameraVision {

    /*
        Constructor that allows user to choose which camera to use
     */
    public CameraVision(VuforiaLocalizer.CameraDirection cameraDirection){
        this.cameraDirection = cameraDirection;
    }

    /*
        Default constructor
     */
    public CameraVision(){

    }

    //Flag for whether Vuforia should be running or not
    boolean vuforiaRunning = false;

    //Paramters for Vuforia initializtion to be used
    private VuforiaLocalizer.Parameters params = null;
    private VuforiaLocalizer.CameraDirection cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
    private VuforiaLocalizer vuforia = null;

    //Variables that will handle a series of images to be tracked
    private VuforiaTrackables beacons = null;
    private ImageData[] imageData = null;

    /**
     * A subclass that handles data from an image including its name, produced matrix,
     * translation vector, and specific matrix values that determines image orientation
     *
     * X-component of VectorF = translation.get(0)
     *  - Positive numbers toward the left
     * Y-component of VectorF = translation.get(1)
     *  - Positive numbers downwards
     * Z-component of VectorF = translation.get(2)
     *  - Distance = abs(negative number)
     *
     * perpendiularness = matrix.getData()[0]
     *  - Range: 0 to 1
     *  - 0: camera view is parallel to image
     *  - 1: camera view is perpendicular to image
     *
     * degreesToTurn = matrix.getData()[8]
     *  - Range: -1 to 1
     *  - If < 0, camera is looking at image from left
     *  - If > 0, camera is looking at image from right
     *
     */
    private class ImageData{
        String imageName;
        OpenGLMatrix matrix;
        VectorF translation;
        float perpendicularness;
        float degreesToTurn;
    }

    Runnable cameraThread = new Runnable() {
        @Override
        public void run() {
            while(!Thread.interrupted()) {
                if (vuforiaRunning){
                    runImageTracking();
                }
            }
        }
    };

    /**
     * Vuforia is initialized with pre-created parameters
     */
    public void vuforiaInit() {
        params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = cameraDirection;
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
    }

    /**
     * Function that runs the image tracking during opMode
     */
    private void runImageTracking() {
        for (int i=0; i < beacons.size(); i++) {
            imageData[i].imageName = beacons.get(i).getName();
            imageData[i].matrix = ((VuforiaTrackableDefaultListener) beacons.get(i).getListener()).getPose();
            if (imageData[i].matrix != null) {
                imageData[i].translation = imageData[i].matrix.getTranslation();
                imageData[i].perpendicularness = imageData[i].matrix.getData()[0];
                imageData[i].degreesToTurn = imageData[i].matrix.getData()[8];
            }
        }
    }

    /**
     * Switches vuforia tracking on and off
     *
     * @param state true for on and false for off
     */
    public void toggleVuforia(boolean state){
        vuforiaRunning = state;
    }

    /**
     * Counts how many images Vuforia has tracked
     *
     * @return the number of images in view and tracked by Vuforia at the moment
     */
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

    /**
     * Getter for the image data object
     *
     * @param index of the image data class array
     * @return the image data object
     */
    public ImageData getImage(int index){
        return imageData[index];
    }

    /**
     * Getter for the image's matrix
     *
     * @param index of the image data class array
     * @return the image's matrix
     */
    public OpenGLMatrix getMatrix(int index){
        return getImage(index).matrix;
    }

    /**
     * Getter for the image's translation
     *
     * @param index of the image data class array
     * @return the image's translation
     */
    public VectorF getTranslation(int index){
        return imageData[index].translation;
    }

    /**
     * Gets X clip coordinate of image
     *
     * @param index of the image data class array
     * @return the image's X clip coordinate
     */
    public float getX(int index){
        return getTranslation(index).get(0);
    }

    /**
     * Gets Y clip coordinate of image
     *
     * @param index of the image's Y clip coordinate
     * @return the image's Y clip coordinate
     */
    public float getY(int index){
        return getTranslation(index).get(1);
    }

    /**
     * Get Z axis of the image's translational position
     *
     * @param index of the image
     * @return the Z clip coordinate
     */
    public float getZ(int index){
        return getTranslation(index).get(2);
    }

    /**
     * Getter for the image's perpendicularness to image
     *
     * @param index of the image data class array
     * @return the image's translation
     */
    public float getPerpendicularness(int index){
        return imageData[index].perpendicularness;
    }

    /**
     * Getter for the image's degrees to turn
     *
     * @param index of the image data class array
     * @return the image's degrees to turn
     */
    public float getDegreesToTurn(int index){
        return imageData[index].degreesToTurn;
    }

    /**
     * Gets the image's data object array index given its name
     *
     * @param imageName The name of target image
     * @return the index of the data object of image
     */
    public int getIndex(String imageName){
        int result = -1;
        for (int i=0; i<beacons.size(); i++){
            if (imageData[i].imageName.equals(imageName)){
                result = i;
            }
        }
        return result;
    }

    /**
     * Getter for beacon name
     * @param index of the beacon image array
     * @return the name that it is associated with
     */
    public String getImageName(int index){
        return beacons.get(index).getName();
    }

    /**
     * Get number of trackable images
     *
     * @return the number of trackables
     */
    public int getTrackableSize(){
        return beacons.size();
    }
}
