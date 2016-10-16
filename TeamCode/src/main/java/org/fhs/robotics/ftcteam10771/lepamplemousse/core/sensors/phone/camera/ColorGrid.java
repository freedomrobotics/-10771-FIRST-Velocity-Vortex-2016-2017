package org.fhs.robotics.ftcteam10771.lepamplemousse.core.sensors.phone.camera;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Color;

import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.Static;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * Accepts one or two arguments to determine a grid the camera should average the
 * colors for and return.
 * <p/>
 * Construct with a size of 1 to somewhat mimic a color sensor. (not recommended;
 * recommended instead is to construct with some grid and take the center cell as
 * the color sensor)
 * <p/>
 * Construct to initialize and start. Call {@link #getGrid} to get the whole grid.
 * Call {@link #getCell} to get a part of the grid. Call {@link #closeGrid()} to
 * close the grid (duh!).
 */
// TODO: 1/1/2016 Organization and style. I also understand some of my numbers might
// TODO:          be backwards and that needs to be accounted for as well.
public class ColorGrid extends Camera {

    //The grid of values
    private GridValue[][] grid = null;
    //The size of the grid.
    private int width, height;

    // The scheduled executor service that runs the separate thread with the updateGrid function.
    private ScheduledExecutorService executorService;
    //The runnable that runs the updateGrid function.
    private Runnable gridUpdate = new Runnable() {
        public void run() {
            updateGrid();
        }
    };

    /**
     * Constructor to create a {@link ColorGrid} based on the phones back camera.
     * Will create a square grid given a single side.
     *
     * @param side          Length of one side of the grid.
     * @param refreshRate   How many times per second to refresh the grid.
     * @param context       The context of the app (hardwareMap.appContext)
     */
    public ColorGrid(int side, float refreshRate, Context context){
        this(side, side, refreshRate, context);
    }

    /**
     * Constructor to create a {@link ColorGrid} based on the phones back camera.
     * Will create a rectangular grid given the length of two sides
     *
     * @param width         Width of the grid.
     * @param height        Height of the grid.
     * @param refreshRate   How many times per second to refresh the grid.
     * @param context       The context of the app (hardwareMap.appContext)
     */
    public ColorGrid(int width, int height, float refreshRate, Context context){
        grid = new GridValue[width][height];
        for (int w = 0; w < width; w++){
            for (int h = 0; h < height; h++){
                grid[w][h] = new GridValue();
            }
        }
        this.width = width;
        this.height = height;
        //calculate the nearest downsampling ratio for the camera and set it to the second nearest
        // (unless it's a really large ratio).
        CameraObject.Downsample ds;
        int gridGreatestSide = (width > height) ? width : height;
        int cameraLeastSide = (cameraFullX < cameraFullY) ? cameraFullX : cameraFullY;
        int w = cameraLeastSide/gridGreatestSide;
        int c = (int) Math.floor(Math.log(w) / Math.log(2)) - 1;      //log-base2
        switch (c){
            case 0:
                ds = CameraObject.Downsample.FULL;
                break;
            case 1:
                ds = CameraObject.Downsample.HALF;
                break;
            case 2:
                ds = CameraObject.Downsample.FOURTH;
                break;
            case 3:
                ds = CameraObject.Downsample.EIGHTH;
                break;
            case 4:
                ds = CameraObject.Downsample.SIXTEENTH;
                break;
            case 5:
                ds = CameraObject.Downsample.BY32;
                break;
            case 6:
                ds = CameraObject.Downsample.BY64;
                break;
            default:
                if (c < 0) {
                    ds = CameraObject.Downsample.FULL;
                    break;
                }
                ds = CameraObject.Downsample.BY128;
                break;
        }
        //initialize the camera with the downsample.
        //Bad style ignoring the return.
        createCameraObject(context, ds);
        //and create the thread and the timed intervals.
        executorService = Executors.newScheduledThreadPool(1);
        executorService.scheduleAtFixedRate(gridUpdate, (long)((1.0f / refreshRate) / Static.nanoSecondsToSeconds), (long)((1.0f / refreshRate) / Static.nanoSecondsToSeconds), TimeUnit.NANOSECONDS);

    }

    //Simple function to iteratively update the grid
    //An alternative would've been to create individual threads for each cell or use something like 4 threads to update a section of the grid.
    private void updateGrid(){
        Bitmap preview = cameraObject.cameraData.getRgbImage();
        if (preview == null) return;
        //It's fine to lose a pixel. This just makes sure that I can't exceed the width or height of the bitmap in the next part.
        int cellX = (preview.getWidth() / width) - 1;
        int cellY = (preview.getHeight() / width) - 1;
        for (int w = 0; w < width; w++){
            for (int h = 0; h < height; h++){
                //Annoying one liners.
                grid[w][h].setColor(getAverageColor(Bitmap.createBitmap(preview, w * cellX, h * cellY, cellX, cellY)));
            }
        }
    }

    //http://stackoverflow.com/a/25071613
    private int getAverageColor(Bitmap bitmap) {
        if (null == bitmap) return Color.TRANSPARENT;

        int redBucket = 0;
        int greenBucket = 0;
        int blueBucket = 0;
        int alphaBucket = 0;

        boolean hasAlpha = bitmap.hasAlpha();
        int pixelCount = bitmap.getWidth() * bitmap.getHeight();
        int[] pixels = new int[pixelCount];
        bitmap.getPixels(pixels, 0, bitmap.getWidth(), 0, 0, bitmap.getWidth(), bitmap.getHeight());

        for (int y = 0, h = bitmap.getHeight(); y < h; y++)
        {
            for (int x = 0, w = bitmap.getWidth(); x < w; x++)
            {
                int color = pixels[x + y * w]; // x + y * width
                redBucket += (color >> 16) & 0xFF; // Color.red
                greenBucket += (color >> 8) & 0xFF; // Color.greed
                blueBucket += (color & 0xFF); // Color.blue
                if (hasAlpha) alphaBucket += (color >>> 24); // Color.alpha
            }
        }

        return Color.argb(
                (hasAlpha) ? (alphaBucket / pixelCount) : 255,
                redBucket / pixelCount,
                greenBucket / pixelCount,
                blueBucket / pixelCount);
    }

    /**
     * Returns the {@link GridValue} of a single cell in the ColorGrid
     * @param x         The X value (along the width) of the cell in the grid.
     * @param y         The Y value (along the height) of the cell in the grid.
     * @return          The {@link GridValue} of the given cell in the ColorGrid.
     */
    public GridValue getCell(int x, int y){
        return grid[x][y];
    }

    /**
     * Returns the {@link GridValue} array that makes up the ColorGrid
     * @return The multidimensional {@link GridValue} array that makes up the ColorGrid
     */
    public GridValue[][] getGrid(){
        return grid;
    }

    /**
     * Stops the grid.
     */
    public void closeGrid(){
        //Must stop the scheduled thread! or else! (nothing will happen, but it's still better to do so)
        executorService.shutdown();
        //Stop this "instance" of a camera object
        stopCamera();
    }

    /**
     * <p>A simple container class that holds the RGB values of a given cell in the grid.</p>
     * <p/>
     * Inside:
     * <ul>
     * <li>{@link #red()} Returns the red value of the cell.
     * <li>{@link #green()} Returns the green value of the cell.
     * <li>{@link #blue()} Returns the blue value of the cell.
     * <li>{@link #colorInt()} Returns an integer for use with any function of the {@link android.graphics.Color} class.
     * </ul>
     */
    public class GridValue{
        private int r, g, b;

        GridValue(){
            r = 0;
            g = 0;
            b = 0;
        }

        protected void setColor(int c){
            r = Color.red(c);
            g = Color.green(c);
            b = Color.blue(c);
        }

        /**
         * @return The red value of the cell.
         */
        public int red(){
            return r;
        }

        /**
         * @return The green value of the cell.
         */
        public int green() {
            return g;
        }

        /**
         * @return The blue value of the cell.
         */
        public int blue() {
            return b;
        }

        /**
         * @return An integer for use with any function of the {@link android.graphics.Color} class.
         */
        public int colorInt(){
            return Color.rgb(r, g, b);
        }

        @Override
        public String toString(){
            return "r" + r + ",g" + g + ",b" + b;
        }

    }
}
