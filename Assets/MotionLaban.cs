using System;
// using System.Numerics;
using System.Threading;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MotionLaban : MonoBehaviour
{
    public Camera UserCamera;
    private GameObject Bot;
    private Vector3 screenPos;
    private Queue<Vector3> bufferQueue = new Queue<Vector3>();
    private Vector3[] bufferArray;
    private Vector3 velocityVector;
    private Vector3 accelerationVector;
    private Vector3 jerkVector;
    private int effortQueueInterval;
    private Queue<float> weightQueue = new Queue<float>();
    private Queue<float> timeQueue = new Queue<float>();
    private Queue<Vector3> spaceNumeratorQueue = new Queue<Vector3>();
    private Queue<float> flowQueue = new Queue<float>();
    private int bufferSize = 5;
    private float interval = 0.1f; // Sampling interval in seconds
    private float speed;
    private float speedMax = 1;
    private float acceleration;
    private float accelerationMax = 1;
    private float jerk;
    private float jerkMax = 1;
    private float curvature;
    private float curvatureMax = 1;
    private float alphaWeight = 1; // Alpha weight for joint(s)
    private float weight;
    private float weightMax = 1;
    private float delta = 1; // Time window for time, space and flow efforts calculations in seconds
    private float time;
    private float timeMax = 1;
    private float space;
    private float spaceMax = 1;
    private float flow;
    private float flowMax = 1;
    
    void Start()
    {
        Bot = GameObject.Find("Bot");
        UserCamera = Camera.main;
        StartCoroutine(ReadPosition());
    }

    IEnumerator ReadPosition()
    {
        for(;;) 
        {
            screenPos = UserCamera.WorldToScreenPoint(Bot.transform.position);
            // Remove oldest data from the begginig of the buffer if it is full
            if (bufferQueue.Count == bufferSize) bufferQueue.Dequeue();
            // Add new data to the end of the buffer
            bufferQueue.Enqueue(screenPos);
            // Check if buffer is full before starting the calculations
            if (bufferQueue.Count == bufferSize) CalculateDescriptors();
            // Interval (in seconds) at which new position data should be pushed to buffer
            yield return new WaitForSeconds(interval);
        }
    }

    void CalculateDescriptors()
    {
        // Convert queue to array
        bufferArray = bufferQueue.ToArray();

        // Low-level descriptors
        CalculateSpeed();
        CalculateAcceleration();
        CalculateJerk();
        CalculateCurvature();

        // High-level descriptors - efforts

        // First calculate how many measurements should be bufferd within set time window
        effortQueueInterval = (int)(delta / interval);

        CalculateWeight();
        CalculateTime();
        CalculateSpace();
        CalculateFlow();
    }

    void CalculateSpeed()
    {
        velocityVector.x = (bufferArray[3].x - bufferArray[1].x) / 2 * interval;
        velocityVector.y = (bufferArray[3].y - bufferArray[1].y) / 2 * interval;
        velocityVector.z = (bufferArray[3].z - bufferArray[1].z) / 2 * interval;
        speed = velocityVector.magnitude;

        var result = ScaleValue(speed, speedMax);
        speed = result.Item1;
        speedMax = result.Item2;
        // Debug.Log("Speed: " + speed);
    }

    void CalculateAcceleration()
    {
        accelerationVector.x = (bufferArray[3].x - 2 * bufferArray[2].x + bufferArray[1].x) / Convert.ToSingle(Math.Pow(interval,2));
        accelerationVector.y = (bufferArray[3].y - 2 * bufferArray[2].y + bufferArray[1].y) / Convert.ToSingle(Math.Pow(interval,2));
        accelerationVector.z = (bufferArray[3].z - 2 * bufferArray[2].z + bufferArray[1].z) / Convert.ToSingle(Math.Pow(interval,2));
        acceleration = accelerationVector.magnitude;

        var result = ScaleValue(acceleration, accelerationMax);
        acceleration = result.Item1;
        accelerationMax = result.Item2;
        // Debug.Log("Acceleration: " + acceleration);
    }

    void CalculateJerk()
    {
        jerkVector.x = (bufferArray[4].x - 2 * bufferArray[3].x + 2 * bufferArray[1].x - bufferArray[0].x) / Convert.ToSingle(2 * Math.Pow(interval,3));
        jerkVector.y = (bufferArray[4].y - 2 * bufferArray[3].y + 2 * bufferArray[1].y - bufferArray[0].y) / Convert.ToSingle(2 * Math.Pow(interval,3));
        jerkVector.z = (bufferArray[4].z - 2 * bufferArray[3].z + 2 * bufferArray[1].z - bufferArray[0].z) / Convert.ToSingle(2 * Math.Pow(interval,3));
        jerk = jerkVector.magnitude;

        var result = ScaleValue(jerk, jerkMax);
        jerk = result.Item1;
        jerkMax = result.Item2;
        // Debug.Log("Jerk: " + jerk);
    }

    void CalculateCurvature()
    {
        Vector3 curvatureCross = Vector3.Cross(accelerationVector, velocityVector);
        curvature = Convert.ToSingle(curvatureCross.magnitude / Math.Pow(speed,3));

        var result = ScaleValue(curvature, curvatureMax);
        curvature = result.Item1;
        curvatureMax = result.Item2;
        // Debug.Log("Curvature: " + curvature);
    }

    void CalculateWeight()
    {
        // Energy is the sum of weights for all (two) joints
        float energy = alphaWeight * velocityVector.sqrMagnitude + alphaWeight * velocityVector.sqrMagnitude;
        // Weight is the max value from all values measured within the time window
        if (weightQueue.Count == effortQueueInterval) weightQueue.Dequeue();
        weightQueue.Enqueue(energy);

        float[] weightArray = weightQueue.ToArray();
        weight = Mathf.Max(weightArray);

        var result = ScaleValue(weight, weightMax);
        weight = result.Item1;
        weightMax = result.Item2;
        // Debug.Log("Weight: " + weight);
    }

    void CalculateTime()
    {
        if (timeQueue.Count == effortQueueInterval) timeQueue.Dequeue();
        timeQueue.Enqueue(acceleration);

        float[] timeArray = timeQueue.ToArray();
        time = 0;
        Array.ForEach(timeArray, i => time += i);

        var result = ScaleValue(time, timeMax);
        time = result.Item1;
        timeMax = result.Item2;
        // Debug.Log("Time: " + time);
    }

    void CalculateSpace()
    {
        Vector3 spaceNumeratorVector;
        spaceNumeratorVector.x = Mathf.Abs(bufferArray[2].x - bufferArray[1].x);
        spaceNumeratorVector.y = Mathf.Abs(bufferArray[2].y - bufferArray[1].y);
        spaceNumeratorVector.z = Mathf.Abs(bufferArray[2].z - bufferArray[1].z);

        if (spaceNumeratorQueue.Count == effortQueueInterval) spaceNumeratorQueue.Dequeue();
        spaceNumeratorQueue.Enqueue(spaceNumeratorVector);

        if (spaceNumeratorQueue.Count == effortQueueInterval)
        {
            Vector3[] spaceNumeratorArray = spaceNumeratorQueue.ToArray();

            Vector3 spaceNumeratorSumVector = Vector3.zero;
            Array.ForEach(spaceNumeratorArray, i => spaceNumeratorSumVector += i);

            Vector3 spaceDenominatorVector;
            spaceDenominatorVector.x = Mathf.Abs(spaceNumeratorArray[0].x - spaceNumeratorArray[effortQueueInterval- 1].x);
            spaceDenominatorVector.y = Mathf.Abs(spaceNumeratorArray[0].y - spaceNumeratorArray[effortQueueInterval- 1].y);
            spaceDenominatorVector.z = Mathf.Abs(spaceNumeratorArray[0].z - spaceNumeratorArray[effortQueueInterval -1].z);

            Vector3 spaceVector;
            spaceVector.x = spaceNumeratorSumVector.x / spaceDenominatorVector.x;
            spaceVector.y = spaceNumeratorSumVector.y / spaceDenominatorVector.y;
            spaceVector.z = spaceNumeratorSumVector.z / spaceDenominatorVector.z;
            space = spaceVector.magnitude;

            var result = ScaleValue(space, spaceMax);
            space = result.Item1;
            spaceMax = result.Item2;
        };
        // Debug.Log("Space: " + space);
    }

    void CalculateFlow()
    {
        if (flowQueue.Count == effortQueueInterval) flowQueue.Dequeue();
        flowQueue.Enqueue(jerk);

        float[] flowArray = flowQueue.ToArray();
        flow = 0;
        Array.ForEach(flowArray, i => flow += i);

        var result = ScaleValue(flow, flowMax);
        flow = result.Item1;
        flowMax = result.Item2;
        // Debug.Log("Flow: " + flow);
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
