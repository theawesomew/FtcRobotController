package org.firstinspires.ftc.teamcode.lib;

public class PID {

    private double pConstant, iConstant, dConstant;
    private double errorIntegral = 0, previousError = 0, currentError = 0;
    private double maximumInput = 1, minimumInput = -1;

    public PID (double pConstant, double iConstant, double dConstant, double maximumInput, double minimumInput) {
        this.pConstant = pConstant;
        this.iConstant = iConstant;
        this.dConstant = dConstant;
        this.maximumInput = maximumInput;
        this.minimumInput = minimumInput;
    }

    public double errorCorrection (double sp, double pv) {

        // to be continued

        this.currentError = sp - pv;

        if (Math.abs(this.currentError) > (this.maximumInput - this.minimumInput)/2) {
            this.currentError -= (this.maximumInput-this.minimumInput)*(Math.abs(this.currentError)/this.currentError);
        }

        if ((this.errorIntegral + this.currentError)*this.iConstant < (this.maximumInput - this.minimumInput)/2) {
            this.errorIntegral += this.currentError;
        }

        double u = this.pConstant * this.currentError + this.iConstant * this.errorIntegral + this.dConstant * (this.currentError-this.previousError);
        this.previousError = this.currentError;
        return u;
    }
}
