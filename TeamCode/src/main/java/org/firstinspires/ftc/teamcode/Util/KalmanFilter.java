package org.firstinspires.ftc.teamcode.Util;

public class KalmanFilter {
    double state, mV, sV, eV, kGain, pState, pVariance;

    public KalmanFilter(double modelCovariance, double sensorCovariance) {
        state = 0;
        mV = modelCovariance;
        sV = sensorCovariance;
    }

    public double estimate(double measurement, double model) {

        pVariance = eV;

        state = pState + measurement;

        eV = pVariance + mV;

        kGain = eV / (eV + sV);

        state = pState + kGain * (model - state);

        eV = (1 - kGain) * eV;

        return state;
    }

}
