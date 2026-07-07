package org.firstinspires.ftc.teamcode;

public class Constants {
    public class DriveTrain {
        public class ForwardPod{
            public static final double X = 0.1; // meters
            public static final double Y = 0.1; // meters
            public class turnMotor {
                public static final double TICKS_PER_REV = 537.6; // ticks per revolution
                public static final double GEAR_RATIO = 1.0; // gear ratio
                public static final double WHEEL_DIAMETER = 0.05; // meters
                public static final double TICKS_PER_METER = (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER); // ticks per meter
                public static final double p = 0.1; // PID coefficients
            }
            public class driveMotor {
                public static final double TICKS_PER_REV = 537.6; // ticks per revolution
                public static final double GEAR_RATIO = 1.0; // gear ratio
                public static final double WHEEL_DIAMETER = 0.05; // meters
                public static final double TICKS_PER_METER = (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER); // ticks per meter
                public static final double p = 0.1; // NEED TO FIGURE OUT LATER
                public static final double i = 0.1; // NEED TO FIGURE OUT LATER
                public static final double d = 0.1; // NEED TO FIGURE OUT LATER
                public static final double f = 0.1; // NEED TO FIGURE OUT LATER
            }

        }
        public class BackPod{
            public static final double X = -0.1; // meters
            public static final double Y = -0.1; // meters

            public class turnMotor {
                public static final double TICKS_PER_REV = 537.6; // ticks per revolution
                public static final double GEAR_RATIO = 1.0; // gear ratio
                public static final double WHEEL_DIAMETER = 0.05; // meters
                public static final double TICKS_PER_METER = (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER); // ticks per meter
                public static final double p = 0.1; // PID coefficients
            }
            public class driveMotor {
                public static final double TICKS_PER_REV = 537.6; // ticks per revolution
                public static final double GEAR_RATIO = 1.0; // gear ratio
                public static final double WHEEL_DIAMETER = 0.05; // meters
                public static final double TICKS_PER_METER = (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER); // ticks per meter
                public static final double p = 0.1; // NEED TO FIGURE OUT LATER
                public static final double i = 0.1; // NEED TO FIGURE OUT LATER
                public static final double d = 0.1; // NEED TO FIGURE OUT LATER
                public static final double f = 0.1; // NEED TO FIGURE OUT LATER
            }
        }

        public static final double MAX_SPEED = 1.0; // m/s
        public static final double MAX_ANGULAR_SPEED = Math.PI; // rad/s
    }

}
