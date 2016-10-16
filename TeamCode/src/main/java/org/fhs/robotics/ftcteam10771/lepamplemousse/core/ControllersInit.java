package org.fhs.robotics.ftcteam10771.lepamplemousse.core;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.fhs.robotics.ftcteam10771.lepamplemousse.config.Controllers;
import org.fhs.robotics.ftcteam10771.lepamplemousse.core.vars.ReturnValues;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

/**
 * Dynamic aliasing of various variables to the controller based on configuration
 */
public class ControllersInit {

    //region Variables
    // The gamepads
    private Gamepad gamepad1 = null;
    private Gamepad gamepad2 = null;
    // The Controller config
    private Controllers controllerConfig = null;
    //endregion

    private Map<String, inputMethods> aliasing = new HashMap<String, inputMethods>();

    /**
     * Contructs the ControllersInit class which is for the aliasing of various inputs from the gamepads
     *
     * @param gamepad1 Gamepad1
     * @param gamepad2 Gamepad2
     */
    //The constructor takes the two gamepads as input arguments to assign to the class's variable
    //It also constructs the configuration file
    public ControllersInit(Gamepad gamepad1, Gamepad gamepad2, Controllers controllerConfig) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.controllerConfig = controllerConfig;
    }

    public ReturnValues initialize() {
        //Insert code here to alias variables
        /* todo: Think about other way for another lifetime
        for(int i = 1; i <= 2; i++) {
            Iterator Gamepad = controllerConfig.getEntrySet(i).iterator();
            somthing soemthins galsrhehaiufsadf
        }
        */
        //Some similar but more cheaply thrown together way
        for (int i = 1; i <= 2; i++) {
            Iterator Gamepad = controllerConfig.getEntrySet(i).iterator();
            while (Gamepad.hasNext()) {
                Map.Entry button = (Map.Entry) Gamepad.next();
                String key = button.getKey().toString();
                if (controllerConfig.getFunction(i, key) != null) {
                    String functionNames = controllerConfig.getFunction(i, key);
                    for(String functionName : functionNames.split(" ")){
                        boolean inverted = controllerConfig.invertedEnabled(i, key);
                        if (controllerConfig.digitalEnabled(i, key)) {
                            aliasing.put(functionName, new inputMethods(key, i, inverted, true));
                        } else {
                            aliasing.put(functionName, new inputMethods(key, i, inverted, false));
                            //What? Use default? Will do later
                            // TODO: 12/16/2015 Appropriate return values
                        }
                    }
                }
            }
        }
        return ReturnValues.SUCCESS;
    }

    private Object getGamepad(Integer id, String name) {
        if (name.equals("left_stick_x")) {
            if (id.equals(1)) {
                return gamepad1.left_stick_x;
            }
            if (id.equals(2)) {
                return gamepad2.left_stick_x;
            }
        }
        if (name.equals("left_stick_y")) {
            if (id.equals(1)) {
                return gamepad1.left_stick_y;
            }
            if (id.equals(2)) {
                return gamepad2.left_stick_y;
            }
        }
        if (name.equals("right_stick_x")) {
            if (id.equals(1)) {
                return gamepad1.right_stick_x;
            }
            if (id.equals(2)) {
                return gamepad2.right_stick_x;
            }
        }
        if (name.equals("right_stick_y")) {
            if (id.equals(1)) {
                return gamepad1.right_stick_y;
            }
            if (id.equals(2)) {
                return gamepad2.right_stick_y;
            }
        }
        if (name.equals("dpad_up")) {
            if (id.equals(1)) {
                return gamepad1.dpad_up;
            }
            if (id.equals(2)) {
                return gamepad2.dpad_up;
            }
        }
        if (name.equals("dpad_down")) {
            if (id.equals(1)) {
                return gamepad1.dpad_down;
            }
            if (id.equals(2)) {
                return gamepad2.dpad_down;
            }
        }
        if (name.equals("dpad_left")) {
            if (id.equals(1)) {
                return gamepad1.dpad_left;
            }
            if (id.equals(2)) {
                return gamepad2.dpad_left;
            }
        }
        if (name.equals("dpad_right")) {
            if (id.equals(1)) {
                return gamepad1.dpad_right;
            }
            if (id.equals(2)) {
                return gamepad2.dpad_right;
            }
        }
        if (name.equals("a_button")) {
            if (id.equals(1)) {
                return gamepad1.a;
            }
            if (id.equals(2)) {
                return gamepad2.a;
            }
        }
        if (name.equals("b_button")) {
            if (id.equals(1)) {
                return gamepad1.b;
            }
            if (id.equals(2)) {
                return gamepad2.b;
            }
        }
        if (name.equals("x_button")) {
            if (id.equals(1)) {
                return gamepad1.x;
            }
            if (id.equals(2)) {
                return gamepad2.x;
            }
        }
        if (name.equals("y_button")) {
            if (id.equals(1)) {
                return gamepad1.y;
            }
            if (id.equals(2)) {
                return gamepad2.y;
            }
        }
        if (name.equals("guide_button")) {
            if (id.equals(1)) {
                return gamepad1.guide;
            }
            if (id.equals(2)) {
                return gamepad2.guide;
            }
        }
        if (name.equals("start_button")) {
            if (id.equals(1)) {
                return gamepad1.start;
            }
            if (id.equals(2)) {
                return gamepad2.start;
            }
        }
        if (name.equals("back_button")) {
            if (id.equals(1)) {
                return gamepad1.back;
            }
            if (id.equals(2)) {
                return gamepad2.back;
            }
        }
        if (name.equals("left_bumper")) {
            if (id.equals(1)) {
                return gamepad1.left_bumper;
            }
            if (id.equals(2)) {
                return gamepad2.left_bumper;
            }
        }
        if (name.equals("right_bumper")) {
            if (id.equals(1)) {
                return gamepad1.right_bumper;
            }
            if (id.equals(2)) {
                return gamepad2.right_bumper;
            }
        }
        if (name.equals("left_stick_button")) {
            if (id.equals(1)) {
                return gamepad1.left_stick_button;
            }
            if (id.equals(2)) {
                return gamepad2.left_stick_button;
            }
        }
        if (name.equals("right_stick_button")) {
            if (id.equals(1)) {
                return gamepad1.right_stick_button;
            }
            if (id.equals(2)) {
                return gamepad2.right_stick_button;
            }
        }
        if (name.equals("left_trigger")) {
            if (id.equals(1)) {
                return gamepad1.left_trigger;
            }
            if (id.equals(2)) {
                return gamepad2.left_trigger;
            }
        }
        if (name.equals("right_trigger")) {
            if (id.equals(1)) {
                return gamepad1.right_trigger;
            }
            if (id.equals(2)) {
                return gamepad2.right_trigger;
            }
        }
        return null;
    }

    public Float getAnalog(String name) {
        if (aliasing.containsKey(name))
            return aliasing.get(name).getFloat();
        return 0.0f;
    }

    public Boolean getDigital(String name) {
        if (aliasing.containsKey(name))
            return aliasing.get(name).getBoolean();
        return false;
    }

    class inputMethods {
        boolean inverted = false;
        boolean digital = false;
        Integer id = 0;
        String name;

        inputMethods(String name, Integer id, boolean inverted, boolean digital) {
            this.name = name;
            this.id = id;
            this.inverted = inverted;
            this.digital = digital;

        }

        public boolean getBoolean() {
            if (digital && (getGamepad(id, name) != null)) {
                return inverted ^ (Boolean) getGamepad(id, name);
            }
            return false;
        }

        public float getFloat() {
            if (!digital && (getGamepad(id, name) != null)) {
                if (inverted) {
                    return -(Float) getGamepad(id, name);
                }
                return (Float) getGamepad(id, name);
            }
            return 0.0f;
        }
    }
}