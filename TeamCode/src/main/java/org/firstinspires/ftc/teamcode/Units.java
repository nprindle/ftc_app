package org.firstinspires.ftc.teamcode;

// This enumeration will eventually replace the hardcoded
// units in AutonomousTemplate
public enum Units {
    IN(1.0),
    FT(12.0),
    M(39.37),
    CM(0.3937);

    // contains the ratio of inches to the given unit
    // that is, to determine the number of inches from
    // the number of any unit, it would simple look like
    // unit * inchRatio
    private double inchRatio;

    Units(double inchRatio) {
        this.inchRatio = inchRatio;
    }

    public double convertToInches(double amount) {
        return amount * inchRatio;
    }

}
