package com.example.meepmeeptesting;

public class Hardware {
    class Servo {
        private Direction direction;
        public static enum Direction{
            FORWARD,
            REVERSE
        }
        static final double MAX_POSITION = 1.0;
        static final double MIN_POSITION = 1.0;
        Direction getDirection(){
            return direction;
        }
    }
}
