package org.firstinspires.ftc.teamcode.config.util.logging;

public enum LogType {
    //    ARM_POSITION("Arm Position", "radians"),
//    EXTENSION_POSITION("Extension Position", "ticks"),
//    CONTOUR_LENGTH("Contour Length", "px"),
//    CONTOUR_AREA("Contour Area", "px"),
//    CENTROID_X("Centroid X", "px"),
//    CENTROID_Y("Centroid Y", "px"),
//    TAPE_CONTOUR_LENGTH("Tape Contour Length", "px"),
//    TAPE_CONTOUR_AREA("Tape Contour Area", "px"),
//    TAPE_CENTROID_X("Tape Centroid X", "px"),
//    TAPE_CENTROID_Y("Tape Centroid Y", "px");
    TURRET_TARGET("turret target", "degrees"),
    TURRET_VOLTS("turret raw", "volts"),
    TURRET_PREV("turret prev", "volts"),
    TURRET_FULL_ROTS("turret full rotations", "rotations"),
    LAUNCHER_VELOCITY("current vel", "rpm"),
    LAUNCHER_TARGET("target vel", "rpm"),
    LAUNCHER_SETTLED("launcher done", "boolean"),
    ROBOT_X("x", "in"),
    ROBOT_Y("y", "in"),
    ROBOT_HEADING("heading", "degrees"),
    INTAKE_POWER("Intake power", "power"),
    BOOST("Boost", "boolean"),

    AGGRESSIVE("Aggressive", "boolean"),
    LAUNCHER_POWER("Launcher Power", "Power")
            ;



    private final String header;
    private final String unit;

    LogType(String header, String unit) {
        this.header = header;
        this.unit = unit;
    }

    public String getHeader() {
        return this.header;
    }

    public String getUnit() {
        return this.unit;
    }
}
