package org.firstinspires.ftc.teamcode.helpers;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PIDFlywheel {
    public static double Kp = 2;  // Proportional gain
    public static double Ki = 0.0;   // Integral gain ( KEEP THIS 0 )
    public static double Kd = 0.001;   // Derivative gain
}
