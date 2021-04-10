package org.firstinspires.ftc.teamcode.lib;

import java.lang.Math;

public class Vector {

    private double values[];

    public Vector (double ...inValues) {
        this.values = inValues;
    }

    public Vector (double magnitude, double angle) {
        this.values = new double[]{magnitude * Math.cos(angle), magnitude * Math.sin(angle)};
    }

    public double GetMagnitude () throws Exception {
        return Math.pow(this.Dot(this), (1.0/this.GetDimension()));
    }

    public double GetAngle () throws Exception {
        if (this.GetDimension() != 2) {
            throw new Exception("There is no implementation of the angle of vectors beyond 2 dimensions");
        } else {
            return Math.atan2(this.GetValues()[1], this.GetValues()[0]);
        }
    }

    public double Dot (Vector vector) throws Exception {
        if (!this.CheckSize(vector)) {
            throw new Exception("The dot product of two vectors of different dimensions is being taken");
        } else {
            double product = 0;
            for (int i = 0; i < this.GetDimension(); ++i) {
                product += this.values[i]*vector.values[i];
            }
            return product;
        }
    }

    public double GetAngleBetweenVectors (Vector vector) throws Exception {
        if (!this.CheckSize(vector)) {
            throw new Exception("The dot product of two vectors of different dimensions is being taken");
        } else {
            return Math.acos(this.Dot(vector)/(this.getMagnitude()*vector.getMagnitude()));
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

    public Vector Cross (Vector vector) throws Exception {
        if (vector.GetDimension() != 3) {
            throw new Exception("Taking cross-product of non-three-dimensional vector");
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

    public Vector GetUnit () throws Exception {
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

    public Vector Add (Vector vector) throws Exception {
        if (!this.CheckSize(vector)) {
            throw new Exception("Adding vectors of different dimensions");
        } else {
            double[] newValues = new double[this.GetDimension()-1];
            for (int i = 0; i < this.GetDimension(); ++i) {
                newValues[i] = this.GetValues()[i] + vector.GetValues()[i];
            }

            return new Vector(newValues);
        }
    }

    public Vector Subtract (Vector vector) throws Exception {
        if (!this.CheckSize(vector)) {
            throw new Exception("Subtracting vectors of different dimensions");
        } else {
            double[] newValues = new double[this.GetDimension()-1];
            for (int i = 0; i < this.GetDimension(); ++i) {
                newValues[i] = this.GetValues()[i] - vector.GetValues()[i];
            }

            return new Vector(newValues);
        }
    }
}
