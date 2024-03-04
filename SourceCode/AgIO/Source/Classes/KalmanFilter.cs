/*
 * SimpleKalmanFilter - a Kalman Filter implementation for single variable models.
 * Created by Denys Sene, January, 1, 2017.
 * Ported to C# by chriskinal.
 * Released under MIT License - see LICENSE file for details.
 */

using System;

public class SimpleKalmanFilter
{
    private float _err_measure, _err_estimate, _q, _current_estimate = 0F, _last_estimate = 0F, _kalman_gain = 0F;

    public SimpleKalmanFilter(float mea_e, float est_e, float q)
    {
        _err_measure = mea_e;
        _err_estimate = est_e;
        _q = q;
    }

    public float updateEstimate(float mea)
    {
        _kalman_gain = _err_estimate / (_err_estimate + _err_measure);
        _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
        _err_estimate = (1.0f - _kalman_gain) * _err_estimate + Math.Abs(_last_estimate - _current_estimate) * _q;
        _last_estimate = _current_estimate;

        return _current_estimate;
    }

    public void setMeasurementError(float mea_e)
    {
        _err_measure = mea_e;
    }

    public void setEstimateError(float est_e)
    {
        _err_estimate = est_e;
    }

    public void setProcessNoise(float q)
    {
        _q = q;
    }

    public float getKalmanGain()
    {
        return _kalman_gain;
    }

    public float getEstimateError()
    {
        return _err_estimate;
    }
}
