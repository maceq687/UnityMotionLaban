using System;
// using System.Numerics;
using System.Threading;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Efforts : MonoBehaviour
{
    private float interval = 0.1f; // Sampling interval in seconds
    private Queue<float> weightQueue = new Queue<float>();
    private Queue<float> timeQueue = new Queue<float>();
    private Queue<Vector3> spaceNumeratorQueue = new Queue<Vector3>();
    private Queue<float> flowQueue = new Queue<float>();
    private float alphaWeight = 1; // Alpha weight for joint(s)
    private float weightMax = 1;
    private float delta = 1; // Time window for time, space and flow efforts calculations in seconds
    private float alphaTime = 1; // Alpha time for joint(s)
    private float timeMax = 1;
    private float alphaSpace = 1; // Alpha space for joint(s)
    private float spaceMax = 1;
    private float alphaFlow = 1; // Alpha flow for joint(s)
    private float flowMax = 1;
    

    void Start()
    {
        CalculateMotion("Bot", "Bot", "Bot");
    }


    void CalculateMotion(string leftHandJoint, string rightHandJoint, string headJoint)
    {
        Vector3[] leftHandBufferArray = MotionLaban.JointTracking(leftHandJoint).bufferArray;
        Vector3 leftHandVelocityVector = MotionLaban.JointTracking(leftHandJoint).velocityVector;
        Vector3 leftHandAccelerationVector = MotionLaban.JointTracking(leftHandJoint).accelerationVector;
        float leftHandJerk = MotionLaban.JointTracking(leftHandJoint).jerk;
        float leftHandCurvature = MotionLaban.JointTracking(leftHandJoint).curvature;
        // float leftHandAcceleration = MotionLaban.JointTracking(leftHandJoint).accelerationVector.magnitute;

        Vector3[] rightHandBufferArray = MotionLaban.JointTracking(rightHandJoint).bufferArray;
        Vector3 rightHandVelocityVector = MotionLaban.JointTracking(rightHandJoint).velocityVector;
        Vector3 rightHandAccelerationVector = MotionLaban.JointTracking(rightHandJoint).accelerationVector;
        float rightHandJerk = MotionLaban.JointTracking(rightHandJoint).jerk;
        float rightHandCurvature = MotionLaban.JointTracking(rightHandJoint).curvature;
        // float rightHandAcceleration = MotionLaban.JointTracking(rightHandJoint).accelerationVector.magnitute;

        Vector3[] headBufferArray = MotionLaban.JointTracking(headJoint).bufferArray;
        Vector3 headVelocityVector = MotionLaban.JointTracking(headJoint).velocityVector;
        Vector3 headAccelerationVector = MotionLaban.JointTracking(headJoint).accelerationVector;
        float headJerk = MotionLaban.JointTracking(headJoint).jerk;
        float headCurvature = MotionLaban.JointTracking(headJoint).curvature;
        // float headAcceleration = MotionLaban.JointTracking(headJoint).accelerationVector.magnitute;

        List<float> accelerationTime = new List<int>(leftHandAccelerationVector.magnitude, rightHandAccelerationVector.magnitude, headAccelerationVector.magnitude);
        
        // High-level descriptors - efforts

        // First calculate how many measurements should be bufferd within set time window
        int effortQueueInterval = (int)(delta / interval);

        float weight = CalculateWeight(leftHandVelocityVector, rightHandVelocityVector, headVelocityVector, effortQueueInterval);
        float time = CalculateTime(accelerationTime, effortQueueInterval);
        float space = CalculateSpace(leftHandBufferArray, rightHandBufferArray, headBufferArray, effortQueueInterval);
        float flow = CalculateFlow(leftHandJerk, rightHandJerk, headJerk, effortQueueInterval);
    }

    float CalculateWeight(Vector3 leftHandVelocityVector, Vector3 rightHandVelocityVector, Vector3 headVelocityVector, int effortQueueInterval)
    {
        // Energy is the sum of weights for all (two) joints
        float energy = alphaWeight * leftHandVelocityVector.sqrMagnitude + alphaWeight * rightHandVelocityVector.sqrMagnitude + alphaWeight * headVelocityVector.sqrMagnitude;
        // Weight is the max value from all values measured within the time window
        if (weightQueue.Count == effortQueueInterval) weightQueue.Dequeue();
        weightQueue.Enqueue(energy);

        float[] weightArray = weightQueue.ToArray();
        float weight = Mathf.Max(weightArray);

        var result = ScaleValue(weight, weightMax);
        weight = result.Item1;
        weightMax = result.Item2;
        // Debug.Log("Weight: " + weight);
        return weight;
    }

    float CalculateTime(List<float> accelerationTime, int effortQueueInterval)
    {
        float timeSum = 0;

        foreach (float element in accelerationTime);
        {
            if (timeQueue.Count == effortQueueInterval) timeQueue.Dequeue();
            timeQueue.Enqueue(element);

            foreach (var time in timeQueue)
            {
                timeSum += time;
            }

            // float[] timeArray = timeQueue.ToArray();
            // float time = 0;
            // Array.ForEach(timeArray, i => time += i);

        }

        

        var result = ScaleValue(time, timeMax);
        time = result.Item1;
        timeMax = result.Item2;
        // Debug.Log("Time: " + time);
        return time;
    }

    float CalculateSpace(Vector3[] leftHandBufferArray, Vector3[] rightHandBufferArray, Vector3[] headBufferArray, int effortQueueInterval)
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
            float space = spaceVector.magnitude;

            var result = ScaleValue(space, spaceMax);
            space = result.Item1;
            spaceMax = result.Item2;
        };
        // Debug.Log("Space: " + space);
        return space;
    }

    float CalculateFlow(float leftHandJerk, float rightHandJerk, float headJerk, int effortQueueInterval)
    {
        if (flowQueue.Count == effortQueueInterval) flowQueue.Dequeue();
        flowQueue.Enqueue(jerk);

        float[] flowArray = flowQueue.ToArray();
        float flow = 0;
        Array.ForEach(flowArray, i => flow += i);

        var result = ScaleValue(flow, flowMax);
        flow = result.Item1;
        flowMax = result.Item2;
        // Debug.Log("Flow: " + flow);
        return flow;
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
