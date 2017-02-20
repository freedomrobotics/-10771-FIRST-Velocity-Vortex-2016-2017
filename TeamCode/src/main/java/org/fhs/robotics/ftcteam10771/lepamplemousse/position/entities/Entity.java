package org.fhs.robotics.ftcteam10771.lepamplemousse.position.entities;

import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Coordinate;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.core.Rotation;
import org.fhs.robotics.ftcteam10771.lepamplemousse.position.vector.VectorR;

/**
 * Created by Adam Li on 1/13/2016.
 */

public interface Entity {

    Rotation rotation = new Rotation();
    Position position = new Position();
    Size size = new Size();
    Offset offset = new Offset();

    Rotation getRotation();
    Position getPosition();
    Size getSize();
    Offset getOffset();
    VectorR getVectorR();

    class Position extends Coordinate {
        public void moveX(float x){
            this.x += x;
        }
        public void moveY(float y){
            this.y += y;
        }
        public void moveXY(float x, float y){
            this.x += x;
            this.y += y;
        }
        public void movePolar(float radius, float theta){
            this.x += (float)Math.cos(theta) * radius;
            this.y += (float)Math.sin(theta) * radius;
        }
        public void movePolarDeg(float radius, float theta){
            this.x = (float)Math.cos((theta / (float)Math.PI) * 180.0f) * radius;
            this.y += (float)Math.sin((theta / (float)Math.PI) * 180.0f) * radius;
        }
        public void moveVectorR(VectorR vectorR){
            this.x += vectorR.getX();
            this.y += vectorR.getY();
        }
    }
    class Size extends Coordinate{
        public void scaleX(float x){
            this.x *= x;
        }
        public void scaleY(float y){
            this.y *= y;
        }
        public void scale(float scalar){
            this.x *= scalar;
            this.y *= scalar;
        }
        public void scaleVectorR(VectorR vectorR){
            this.x *= vectorR.getX();
            this.y *= vectorR.getY();
        }
    }
    class Offset extends Coordinate{
        // nothing yet!
    }
}
