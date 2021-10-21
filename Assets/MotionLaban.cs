using System.Runtime.CompilerServices;
using System;
// using System.Numerics;
using System.Threading;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MotionLaban : MonoBehaviour
{
    private Camera Cam = Camera.main;
    private Queue<Vector3> bufferQueue = new Queue<Vector3>();
    private int bufferSize = 5;
    private float interval = 0.1f; // Sampling interval in seconds
    public Vector3[] bufferArray;
    public Vector3 velocityVector;
    public Vector3 accelerationVector;
    public float jerk;
    public float curvature;
    private float speedMax = 1;
    private float accelerationMax = 1;
    private float jerkMax = 1;
    private float curvatureMax = 1;
    

    public void JointTracking(string JointName)
    {
        GameObject Joint = GameObject.Find(JointName);
        StartCoroutine(ReadPosition());
    }

    IEnumerator ReadPosition()
    {
        for(;;) 
        {
            Vector3 screenPos = Cam.WorldToScreenPoint(Joint.transform.position);
            // Remove oldest data from the begginig of the buffer if it is full
            if (bufferQueue.Count == bufferSize) bufferQueue.Dequeue();
            // Add new data to the end of the buffer
            bufferQueue.Enqueue(screenPos);
            // Check if buffer is full before starting the calculations
            if (bufferQueue.Count == bufferSize) CalculateDescriptors(bufferQueue.ToArray());
            // Interval (in seconds) at which new position data should be pushed to buffer
            yield return new WaitForSeconds(interval);
        }
    }

    void CalculateDescriptors(Vector3[] buffer)
    {
        bufferArray = buffer;
        // Low-level descriptors
        velocityVector = CalculateSpeed(bufferArray);
        Vector3 accelerationVector = CalculateAcceleration(bufferArray);
        jerk = CalculateJerk(bufferArray);
        curvature = CalculateCurvature(velocityVector, accelerationVector);
    }

    Vector3 CalculateSpeed(Vector3[] bufferArray)
    {
        Vector3 velocityVector;
        velocityVector.x = (bufferArray[3].x - bufferArray[1].x) / 2 * interval;
        velocityVector.y = (bufferArray[3].y - bufferArray[1].y) / 2 * interval;
        velocityVector.z = (bufferArray[3].z - bufferArray[1].z) / 2 * interval;
        float speed = velocityVector.magnitude;

        var result = ScaleValue(speed, speedMax);
        speed = result.Item1;
        speedMax = result.Item2;
        // Debug.Log("Speed: " + speed);
        return velocityVector;
    }

    Vector3 CalculateAcceleration(Vector3[] bufferArray)
    {
        Vector3 accelerationVector;
        accelerationVector.x = (bufferArray[3].x - 2 * bufferArray[2].x + bufferArray[1].x) / Convert.ToSingle(Math.Pow(interval,2));
        accelerationVector.y = (bufferArray[3].y - 2 * bufferArray[2].y + bufferArray[1].y) / Convert.ToSingle(Math.Pow(interval,2));
        accelerationVector.z = (bufferArray[3].z - 2 * bufferArray[2].z + bufferArray[1].z) / Convert.ToSingle(Math.Pow(interval,2));
        float acceleration = accelerationVector.magnitude;

        var result = ScaleValue(acceleration, accelerationMax);
        acceleration = result.Item1;
        accelerationMax = result.Item2;
        // Debug.Log("Acceleration: " + acceleration);
        return accelerationVector;
    }

    float CalculateJerk(Vector3[] bufferArray)
    {
        Vector3 jerkVector;
        jerkVector.x = (bufferArray[4].x - 2 * bufferArray[3].x + 2 * bufferArray[1].x - bufferArray[0].x) / Convert.ToSingle(2 * Math.Pow(interval,3));
        jerkVector.y = (bufferArray[4].y - 2 * bufferArray[3].y + 2 * bufferArray[1].y - bufferArray[0].y) / Convert.ToSingle(2 * Math.Pow(interval,3));
        jerkVector.z = (bufferArray[4].z - 2 * bufferArray[3].z + 2 * bufferArray[1].z - bufferArray[0].z) / Convert.ToSingle(2 * Math.Pow(interval,3));
        float jerk = jerkVector.magnitude;

        var result = ScaleValue(jerk, jerkMax);
        jerk = result.Item1;
        jerkMax = result.Item2;
        // Debug.Log("Jerk: " + jerk);
        return jerk;
    }

    float CalculateCurvature(Vector3 velocityVector, Vector3 accelerationVector)
    {
        Vector3 curvatureCross = Vector3.Cross(accelerationVector, velocityVector);
        float curvature = Convert.ToSingle(curvatureCross.magnitude / Math.Pow(velocityVector.magnitude,3));

        var result = ScaleValue(curvature, curvatureMax);
        curvature = result.Item1;
        curvatureMax = result.Item2;
        // Debug.Log("Curvature: " + curvature);
        return curvature;
    }

    private static Tuple<float, float> ScaleValue(float value, float valueMax)
    {
        if (value > valueMax)
        { valueMax = value; };
        float result = value / valueMax;
        var tuple = new Tuple<float, float>(result, valueMax);
        return tuple;
    }
}

// TO DO
// - multiply time, space and effort by alpha and sum
// - smoothing?
