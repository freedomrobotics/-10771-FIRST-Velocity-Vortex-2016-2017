package org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera;

import android.provider.ContactsContract;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Coordinate;
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

    public enum Image{

        WHEELS("Wheels", 0, -182.9f, Coordinate.convertTo(3.0f, Coordinate.UNIT.FT_TO_UNIT)),
        TOOLS("Tools", 1, Coordinate.convertTo(-3.0f, Coordinate.UNIT.FT_TO_UNIT), 182.9f),
        LEGOS("Legos", 2, -182.9f, Coordinate.convertTo(-1.0f, Coordinate.UNIT.FT_TO_UNIT)),
        GEARS("Gears", 3, Coordinate.convertTo(1.0f, Coordinate.UNIT.FT_TO_UNIT), 182.9f),
        NULL("Null", 4, 0f, 0f);

        private String name;
        private int index;
        private float xCoordinate;
        private float yCoordinate;

        Image(String name, int index, float xCoordinate, float yCoordinate){
            this.name = name;
            this.index = index;
            this.xCoordinate = xCoordinate;
            this.yCoordinate = yCoordinate;
        };

        public String getName(){
            return name;
        }

        public int getIndex(){
            return index;
        }

        public float getxCoordinate(){
            return xCoordinate;
        }

        public float getyCoordinate(){
            return yCoordinate;
        }

        private Image getImage(int index){
            return this;
        }
    };

    //Flag for whether Vuforia should be running or not
    boolean vuforiaRunning = true;

    //Variables that indicate the targeted image
    private Image targetedImage = Image.NULL;

    //Flag on whether to use Radians(Degrees if false)
    private boolean useRadians = true;

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
     * angleToTurn = matrix.getData()[8]
     *  - Range: -1 to 1
     *  - If < 0, camera is looking at image from left
     *  - If > 0, camera is looking at image from right
     *
     */
    private class ImageData{
        String imageName;
        OpenGLMatrix matrix;
        VectorF translation;
        double angleToTurn;
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
     * Source: FTC Team FIXIT 3491
     * https://www.youtube.com/watch?v=2z-o9Ts8XoE
     */
    public void vuforiaInit() {
        params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = cameraDirection;
        params.vuforiaLicenseKey = "AVj2kiX/////AAAAGV0J5W5oOkUTvP1+IKxrWdIpD63oQV8zSY/+qSNDxkt5zj8tW0N9AK7/3yUJRBlnJx80gStuZcHF7JoiKUNj4JmO6gcyIQn2LWZ/0hL9gFM+PmwM6lvzJu9U/gmvf++GngzR74ft0gjlNPle9qDHEaAgMHYcbEDpc4msHDVn6ZjCcxDem2tQyW4gEY334fwAU9E0ySkw1KwC/Mo6gaE7bW1Mh9xLbXYTe2+sRclEA6YbrKeH8LHmJBDXQxTdcL4HyS26oPYAGRXfFLoi7QkBdkPDYKiPQUsCoHhNz1uhPh5duEdwOD9Sm6YUPZYet7Mo9QP3sxaDlaqY5l2pHYn/tH31Xu9eqLKe2RmNRzgMNaJ9\n";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
        beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(Image.WHEELS.index).setName(Image.WHEELS.name);
        beacons.get(Image.TOOLS.index).setName(Image.TOOLS.name);
        beacons.get(Image.LEGOS.index).setName(Image.LEGOS.name);
        beacons.get(Image.GEARS.index).setName(Image.GEARS.name);
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
                    imageData[i].angleToTurn = Math.asin((double)imageData[i].matrix.getData()[8]);
                    if (!useRadians){
                        imageData[i].angleToTurn = Math.toDegrees(imageData[i].angleToTurn);
                    }
                }
                else {
                    imageData[i].translation = new VectorF(0f, 0f, 0f);
                    imageData[i].angleToTurn = 0f;
                }
            }
        }
    }

    /**
     * User sets to use radians or degrees
     * @param useRadians true for radians, false for degrees
     */
    public void setUnitToRadians(boolean useRadians){
        this.useRadians = useRadians;
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
     * @param image enum id
     * @return the image data object
     */
    public ImageData getImage(Image image){
        return imageData[image.index];
    }

    /**
     * Getter for the image's matrix
     *
     * @param image enum id
     * @return the image's matrix
     */
    public OpenGLMatrix getMatrix(Image image){
        return getImage(image).matrix;
    }

    /**
     * Getter for the image's translation
     *
     * @param image enum id
     * @return the image's translation
     */
    public VectorF getTranslation(Image image){
        return imageData[image.index].translation;
    }

    /**
     * Gets X clip coordinate of image
     *
     * @param image enum id
     * @return the image's X clip coordinate
     */
    public float getX(Image image){
        return getTranslation(image).get(0);
    }

    /**
     * Gets Y clip coordinate of image
     *
     * @param image enum id
     * @return the image's Y clip coordinate
     */
    public float getY(Image image){
        return getTranslation(image).get(1);
    }

    /**
     * Get Z axis of the image's translational position
     *
     * @param image enum id
     * @return the Z clip coordinate
     */
    public float getZ(Image image){
        return getTranslation(image).get(2);
    }

    /**
     * Getter for the image's degrees to turn
     *
     * @param image enum id
     * @return the image's degrees to turn
     */
    public double getAngleToTurn(Image image){
        return imageData[image.index].angleToTurn;
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
     * @param image enum id
     * @return the state of image name's existence
     */
    private boolean imageExists(Image image){
        int match = -1;
        for (int i=0; i<beacons.size(); i++){
            if (imageData[i].imageName.equals(image.name)){
                match = i;
            }
        }
        return (!(match<0));
    }

    /**
     * Determines if the camera can see a specific image
     *
     * @param image enum id
     * @return whether the camera is detecting it
     */
    public boolean imageInSight(Image image){
        if (imageExists(image)){
            return (imageData[image.index].matrix!=null);
        }
        return false;
    }

    /**
     * Sets an image as the target for Vuforia
     * @param image the chosen image to be targeted
     */
    public void setTargetImage(Image image){
        targetedImage = image;
    }

    /**
     * Sets the highest indexed detected image as a target by assigning its index
     * and its string id to the public variables
     */
    public void setADetectedImageAsTarget(){
        Image target = null;
        target = (imageInSight(Image.WHEELS)) ? Image.WHEELS : target;
        target = (imageInSight(Image.TOOLS)) ? Image.TOOLS : target;
        target = (imageInSight(Image.LEGOS)) ? Image.LEGOS : target;
        target = (imageInSight(Image.GEARS)) ? Image.GEARS : target;
        if (target != null){
            setTargetImage(target);
        }
        else setTargetImage(Image.NULL);
    }

    /**
     * Sets the image as target
     * if there is one image in sight
     */
    public void setSingleImageFoundAsTarget(){
        if (countTrackedImages()==1){
            setADetectedImageAsTarget();
        }
        else setTargetImage(Image.NULL);
    }

    /**
     * Getter for the targeted image enumeration
     * @return
     */
    public Image getTargetedImage(){
        return targetedImage;
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
