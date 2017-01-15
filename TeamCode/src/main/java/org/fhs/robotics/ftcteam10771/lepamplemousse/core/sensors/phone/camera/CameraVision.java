package org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
        this.vuforiaInit();
    }

    /*
        Constructor that allows user to choose whether or not
     */
    public CameraVision(boolean vuforiaInit){
        if (vuforiaInit){
            this.vuforiaInit();
        }
    }

    /*
        Constructor that allows user to choose camera side and
        whether to initialize Vuforia instantly
     */
    public CameraVision(VuforiaLocalizer.CameraDirection cameraDirection, boolean vuforiaInit){
        this.cameraDirection = cameraDirection;
        if (vuforiaInit){
            this.vuforiaInit();
        }
    }

    /*
        Default constructor with BACK as default camera
        and initializes vuforia during construction by default
     */
    public CameraVision(){
        this.vuforiaInit();
    }

    //Flag for whether Vuforia should be running or not
    boolean vuforiaRunning = true;

    //Variables that indicate the targeted image
    private int targetedImageIndex;
    private String targetedImageName;

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

    //The thread loop code
    public final Runnable cameraRunnable = new Runnable() {
        @Override
        public void run() {
            while(!Thread.interrupted()) {
                runImageTracking();
            }
        }
    };

    //The respective thread
    public final Thread cameraThread = new Thread(cameraRunnable);

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
    public void runImageTracking() {
        if (vuforiaRunning){
            for (int i=0; i < beacons.size(); i++) {
                imageData[i].imageName = beacons.get(i).getName();
                imageData[i].matrix = ((VuforiaTrackableDefaultListener) beacons.get(i).getListener()).getPose();
                if (imageData[i].matrix != null) {
                    imageData[i].translation = imageData[i].matrix.getTranslation();
                    imageData[i].perpendicularness = imageData[i].matrix.getData()[0];
                    imageData[i].degreesToTurn = imageData[i].matrix.getData()[8];
                }
                else {
                    imageData[i].translation = new VectorF(0f, 0f, 0f);
                    imageData[i].perpendicularness = 0;
                    imageData[i].degreesToTurn = 0;
                }
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

    /**
     * Determines whether the name exists in a trackable image
     * @param image name
     * @return the state of image name's existence
     */
    private boolean imageExists(String image){
        int match = -1;
        for (int i=0; i<beacons.size(); i++){
            if (imageData[i].imageName.equals(image)){
                match = i;
            }
        }
        return (!(match<0));
    }

    /**
     * Determines if the camera can see a specific image
     *
     * @param image name to be detected
     * @return whether the camera is detecting it
     */
    public boolean imageInSight(String image){
        int match = -1;
        if (imageExists(image)){
            for (int i=0; i<beacons.size(); i++){
                if (imageData[i].imageName.equals(image)){
                    match = i;
                }
            }
            return (imageData[match].matrix!=null);
        }
        return false;
    }

    /**
     * Sets the highest indexed detected image as a target by assigning its index
     * and its string id to the public variables
     */
    public void setTargetImage(){
        for (int i=0; i<beacons.size(); i++){
            if (imageData[i].matrix!=null){
                targetedImageIndex = i;
                targetedImageName = imageData[i].imageName;
            }
        }
    }

    /**
     * Sets the image as target
     * if there is one image in sight
     */
    public void setSingleImageFoundAsTarget(){
        if (countTrackedImages()==1){
            setTargetImage();
        }
    }

    /**
     * Sets a target image by name and index
     * @param image
     * @param index
     */
    private void setTargetImage(String image, int index){
        targetedImageName = image;
        targetedImageIndex = index;
    }

    /**
     * Sets the target image by name
     * @param image
     */
    public void setTargetImage(String image){
        if (imageExists(image)){
            setTargetImage(image, getIndex(image));
        }
    }

    /**
     * Sets the target image by index
     * @param index of the image on the array
     */
    public void setTargetImage(int index){
        if (index < beacons.size() && index >= 0){
            setTargetImage(getImageName(index), index);
        }
    }

    /**
     * Getter for image's string ID
     * @return the targeted image's string id
     */
    public String getTargetImageName(){
        return targetedImageName;
    }

    /**
     * Getter for the image's index ID
     * @return the targeted image's index id
     */
    public int getTargetedImageIndex(){
        return targetedImageIndex;
    }

    /**
     * Tests Vuforia in an op mode
     * @param linearOpMode
     */
    public void testVuforia(LinearOpMode linearOpMode){
        toggleVuforia(true);
        for (int i=0; i<beacons.size(); i++) {
            if (imageData[i].matrix != null) {
                linearOpMode.telemetry.addData(imageData[i].imageName, imageData[i].translation);
            } else linearOpMode.telemetry.addData(imageData[i].imageName, "null");
        }
        //toggleVuforia(false) todo: see if this is necessary or not
    }
}
