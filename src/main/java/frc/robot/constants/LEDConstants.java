// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
public class LEDConstants {
    public static final int LEFT_START = 0;
    public static final int RIGHT_START = 37;
    public static final int LEFT_COUNT = 36;
    public static final int RIGHT_COUNT = 66;
    public static class Color {
        public final int red;
        public final int green;
        public final int blue;
        public Color(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }
    }
}