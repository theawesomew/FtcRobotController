package org.firstinspires.ftc.teamcode;

import java.util.HashMap;

public class Vector {

    private double values[];

    public Vector (double ...inValues) {
        this.values = inValues;
    }

    public Vector (double magnitude, double angle) {
        this.values = new double[]{magnitude * Math.cos(angle), magnitude * Math.sin(angle)};
    }

    public double getMagnitude () throws Exception {
        return Math.pow(this.dot(this), (1.0/this.getDimension()));
    }

    public double dot (Vector vector) throws Exception {
        if (!this.checkSize(vector)) {
            throw new Exception("The dot product of two vectors of different dimensions is being taken");
        } else {
            double product = 0;
            for (int i = 0; i < this.getDimension(); ++i) {
                product += this.values[i]*vector.values[i];
            }
            return product;
        }
    }

    public double getAngleBetweenVectors (Vector vector) throws Exception {
        if (!this.checkSize(vector)) {
            throw new Exception("The dot product of two vectors of different dimensions is being taken");
        } else {
            return Math.acos(this.dot(vector)/(this.getMagnitude()*vector.getMagnitude()));
        }
    }

    public double[] getValues () {
        return this.values;
    }

    public int getDimension () {
        return this.getValues().length;
    }

    public boolean checkSize(Vector vector) {
        return (vector.getDimension() == this.getDimension());
    }

    public Vector cross (Vector vector) throws Exception {
        if (vector.getDimension() != 3) {
            throw new Exception("Taking cross-product of non-three-dimensional vector");
        } else {
            double x, y, z;
            double[] a = this.getValues();
            double[] b = vector.getValues();

            x = a[1]*b[2]-a[2]*b[1];
            y = a[2]*b[0]-a[0]*b[2];
            z = a[0]*b[1]-a[1]*b[0];

            return new Vector(x,y,z);
        }
    }

    public Vector getUnit () throws Exception {
        double magnitude = this.getMagnitude();
        if (magnitude > 0) {
            double[] unitValues = {};
            for (int i = 0; i < this.getDimension(); ++i) {
                unitValues[i] = this.values[i] / magnitude;
            }
            return new Vector(unitValues);
        }
        return this;
    }

    public Vector add (Vector vector) throws Exception {
        if (!this.checkSize(vector)) {
            throw new Exception("Adding vectors of different dimensions");
        } else {
            double[] newValues = new double[this.getDimension()-1];
            for (int i = 0; i < this.getDimension(); ++i) {
                newValues[i] = this.getValues()[i] + vector.getValues()[i];
            }

            return new Vector(newValues);
        }
    }

    public Vector subtract (Vector vector) throws Exception {
        if (!this.checkSize(vector)) {
            throw new Exception("Subtracting vectors of different dimensions");
        } else {
            double[] newValues = new double[this.getDimension()-1];
            for (int i = 0; i < this.getDimension(); ++i) {
                newValues[i] = this.getValues()[i] - vector.getValues()[i];
            }

            return new Vector(newValues);
        }
    }
}
