package org.firstinspires.ftc.teamcode.lib;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeleOpMode;

import java.lang.Math;

public class Vector {

    private double values[];

    public static Vector X_2 = new Vector(1,0);

    public Vector (double ...inValues) {
        this.values = inValues;
    }

    public Vector (double magnitude, double angle) {
        this.values = new double[]{magnitude * Math.cos(angle), magnitude * Math.sin(angle)};
    }

    public double GetMagnitude () {
        return Math.pow(this.Dot(this), (1.0/this.GetDimension()));
    }

    public double Dot (Vector vector) {
        if (!this.CheckSize(vector)) {
            return 1; // Why would you ever get this error? You've been careful right?
        } else {
            double product = 0;
            for (int i = 0; i < this.GetDimension(); ++i) {
                product += this.values[i]*vector.values[i];
            }
            return product;
        }
    }

    public double GetAngleBetweenVectors (Vector vector) {
        if (!this.CheckSize(vector)) {
            return 0; // Why would you ever get this error? You've been careful right?
        } else {
            return Math.acos(this.Dot(vector)/(this.GetMagnitude()*vector.GetMagnitude()));
        }
    }

    public double[] GetValues () {
        return this.values;
    }

    public int GetDimension () {
        return this.GetValues().length;
    }

    public boolean CheckSize(Vector vector) {
        return (vector.GetDimension() == this.GetDimension());
    }

    public Vector Cross (Vector vector) {
        if (vector.GetDimension() != 3) {
            return new Vector(1, 1, 1); // Why would you ever get this error? You've been careful right?
        } else {
            double x, y, z;
            double[] a = this.GetValues();
            double[] b = vector.GetValues();

            x = a[1]*b[2]-a[2]*b[1];
            y = a[2]*b[0]-a[0]*b[2];
            z = a[0]*b[1]-a[1]*b[0];

            return new Vector(x,y,z);
        }
    }

    public Vector GetUnit () {
        double magnitude = this.GetMagnitude();
        if (magnitude > 0) {
            double[] unitValues = {};
            for (int i = 0; i < this.GetDimension(); ++i) {
                unitValues[i] = this.values[i] / magnitude;
            }
            return new Vector(unitValues);
        }
        return this;
    }

    public Vector Add (Vector vector, Telemetry telemetry) {
        if (!this.CheckSize(vector)) {
            return new Vector(1, 0); // Why would you ever get this error? You've been careful right?
        } else {
            double[] newValues = new double[this.GetDimension()];
            for (int i = 0; i < this.GetDimension(); i++) {
                telemetry.addData("Add 1: ", this.GetValues()[i]);
                telemetry.addData("Add 2: ", vector.GetValues()[i]);

                newValues[i] = this.GetValues()[i] + vector.GetValues()[i];
            }

            return new Vector(newValues);
        }
    }

    public Vector Subtract (Vector vector) {
        if (!this.CheckSize(vector)) {
            return new Vector(1, 0); // Why would you ever get this error? You've been careful right?
        } else {
            double[] newValues = new double[this.GetDimension()];
            for (int i = 0; i < this.GetDimension(); ++i) {
                newValues[i] = this.GetValues()[i] - vector.GetValues()[i];
            }

            return new Vector(newValues);
        }
    }

    public Vector Scale (double scale) {
        if (scale != 0) {
            double[] newValues = new double[this.GetDimension()];
            for (int i = 0; i < this.GetDimension(); ++i) {
                newValues[i] = this.GetValues()[i] / scale;
            }

            return new Vector(newValues);
        } else {
            return new Vector(1, 0); // Why would you ever get this error? You've been careful right?
        }
    }
}
