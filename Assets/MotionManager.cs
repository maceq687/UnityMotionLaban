using System.Runtime.CompilerServices;
using System;
// using System.Numerics;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MotionManager : MonoBehaviour
{
    private GameObject leftHand;
    private GameObject rightHand;
    private GameObject head;
    private Camera Cam;
    private float interval = 0.1f; // Sampling interval in seconds
    private int t = 5; // Sampling memory (queue) size
    private float delta = 1; // Time window for efforts calculations in seconds
    private int T; // Efforts queue size
    private Queue<Vector3> leftHandPositionQueue = new Queue<Vector3>();
    private Queue<Vector3> rightHandPositionQueue = new Queue<Vector3>();
    private Queue<Vector3> headPositionQueue = new Queue<Vector3>();
    private Queue<Vector3> leftHandAccelQueue = new Queue<Vector3>();
    private Queue<Vector3> rightHandAccelQueue = new Queue<Vector3>();
    private Queue<Vector3> headAccelQueue = new Queue<Vector3>();
    private Queue<Vector3> leftHandJerkQueue = new Queue<Vector3>();
    private Queue<Vector3> rightHandJerkQueue = new Queue<Vector3>();
    private Queue<Vector3> headJerkQueue = new Queue<Vector3>();
    private Queue<float> weightQueue = new Queue<float>();
    private Queue<Vector3> leftHandSpaceNumeratorQueue = new Queue<Vector3>();
    private Queue<Vector3> rightHandSpaceNumeratorQueue = new Queue<Vector3>();
    private Queue<Vector3> headSpaceNumeratorQueue = new Queue<Vector3>();
    private float[] timeAlpha = new float[3] {2,2,1}; // Joints time alphas
    private float[] weightAlpha = new float[3] {2,2,1}; // Joint weight alphas
    private float[] spaceAlpha = new float[3] {2,2,1}; // Joints space alphas
    private float[] flowAlpha = new float[3] {2,2,1}; // joints flow alphas
    private float weightMax = 1;
    private float timeMax = 1;
    private float spaceMax = 1;
    private float flowMax = 1;


    void Start()
    {
        Cam = Camera.main;
        leftHand = GameObject.Find("Bot");
        rightHand = GameObject.Find("Bot");
        head = GameObject.Find("Bot");
        T = (int)(delta / interval);
        JointTracking();
    }

    public void JointTracking()
    {
        StartCoroutine(OnFrame());
    }


    IEnumerator OnFrame()
    {
         for(;;) 
        {
            // Write t last positions to memory
            Vector3 leftHandPosition = Cam.WorldToScreenPoint(leftHand.transform.position);
            InsertIntoMemory(leftHandPosition, leftHandPositionQueue, t);

            Vector3 rightHandPosition = Cam.WorldToScreenPoint(rightHand.transform.position);
            InsertIntoMemory(rightHandPosition, rightHandPositionQueue, t);

            Vector3 headPosition = Cam.WorldToScreenPoint(head.transform.position);
            InsertIntoMemory(headPosition, headPositionQueue, t);

            if (leftHandPositionQueue.Count == t)
            {
                // Calculate velocity
                Vector3 leftHandVelocity = CalculateVelocity(leftHandPositionQueue);
                Vector3 rightHandVelocity = CalculateVelocity(rightHandPositionQueue);
                Vector3 headVelocity = CalculateVelocity(headPositionQueue);

                // Calculate acceleration
                Vector3 leftHandAccel = CalculateAcceleration(leftHandPositionQueue);
                InsertIntoMemory(leftHandAccel, leftHandAccelQueue, t);

                Vector3 rightHandAccel = CalculateAcceleration(rightHandPositionQueue);
                InsertIntoMemory(rightHandAccel, rightHandAccelQueue, t);

                Vector3 headAccel = CalculateAcceleration(headPositionQueue);
                InsertIntoMemory(headAccel, headAccelQueue, t);

                // Calculate jerk
                Vector3 leftHandJerk = CalculateJerk(leftHandPositionQueue);
                InsertIntoMemory(leftHandJerk, leftHandJerkQueue, t);

                Vector3 rightHandJerk = CalculateJerk(rightHandPositionQueue);
                InsertIntoMemory(rightHandJerk, rightHandJerkQueue, t);

                Vector3 headJerk = CalculateJerk(headPositionQueue);
                InsertIntoMemory(headJerk, headJerkQueue, t);

                // Calculate curvature
                float leftHandCurvature = CalculateCurvature(leftHandVelocity, leftHandAccel);
                float rightHandCurvature = CalculateCurvature(rightHandVelocity, rightHandAccel);
                float headCurvature = CalculateCurvature(headVelocity, headAccel);

                // Calculate weight effort
                float energy = weightAlpha[0] * leftHandVelocity.sqrMagnitude + weightAlpha[1] * rightHandVelocity.sqrMagnitude + weightAlpha[2] * headVelocity.sqrMagnitude;
                InsertIntoMemory(energy, weightQueue, T);

                float[] weightArray = weightQueue.ToArray();
                float weight = Mathf.Max(weightArray);

                var weightResult = ScaleValue(weight, weightMax);
                weight = weightResult.Item1;
                weightMax = weightResult.Item2;

                // Calculate time effort
                float leftHandTime = CalculateTime(leftHandAccelQueue);
                float rightHandTime = CalculateTime(rightHandAccelQueue);
                float headTime = CalculateTime(headAccelQueue);

                float time = timeAlpha[0] * leftHandTime + timeAlpha[1] * rightHandTime + timeAlpha[2] * headTime;

                var timeResult = ScaleValue(time, timeMax);
                time = timeResult.Item1;
                timeMax = timeResult.Item2;

                // Calculate space effort
                Vector3 leftHandSpaceNumerator = CalculateSpaceNumerator(leftHandPositionQueue);
                InsertIntoMemory(leftHandSpaceNumerator, leftHandSpaceNumeratorQueue, T);

                Vector3 rightHandSpaceNumerator = CalculateSpaceNumerator(rightHandPositionQueue);
                InsertIntoMemory(rightHandSpaceNumerator, rightHandSpaceNumeratorQueue, T);

                Vector3 headSpaceNumerator = CalculateSpaceNumerator(headPositionQueue);
                InsertIntoMemory(headSpaceNumerator, headSpaceNumeratorQueue, T);

                if (leftHandSpaceNumeratorQueue.Count == T)
                {
                    float leftHandSpace = CalculateSpace(leftHandSpaceNumeratorQueue);
                    float rightHandSpace = CalculateSpace(rightHandSpaceNumeratorQueue);
                    float headSpace = CalculateSpace(headSpaceNumeratorQueue);

                    float space = spaceAlpha[0] * leftHandSpace + spaceAlpha[1] * rightHandSpace + spaceAlpha[2] * headSpace;

                    var spaceResult = ScaleValue(space, spaceMax);
                    space = spaceResult.Item1;
                    spaceMax = spaceResult.Item2;

                    Debug.Log("Space: " + space);
                }
                
                // Calculate flow effort
                float leftHandFlow = CalculateFlow(leftHandJerkQueue);
                float rightHandFlow = CalculateFlow(rightHandPositionQueue);
                float headFlow = CalculateFlow(headJerkQueue);

                float flow = flowAlpha[0] * leftHandFlow + flowAlpha[1] * rightHandFlow + flowAlpha[2] * headFlow;

                var flowResult = ScaleValue(flow, flowMax);
                flow = flowResult.Item1;
                flowMax = flowResult.Item2;

                Debug.Log("Weight: " + weight + " Time: " + time + " Flow: " + flow);
            }

            yield return new WaitForSeconds(interval);
        }
    }

    void InsertIntoMemory<Type>(Type elem, Queue<Type> queue, int size)
    {
        if (queue.Count == size) queue.Dequeue();
        queue.Enqueue(elem);
    }

    Vector3 CalculateVelocity(Queue<Vector3> positionQueue)
    {
        Vector3[] positionArray = positionQueue.ToArray();
        Vector3 velocityVector;
        velocityVector.x = (positionArray[3].x - positionArray[1].x) / 2 * interval;
        velocityVector.y = (positionArray[3].y - positionArray[1].y) / 2 * interval;
        velocityVector.z = (positionArray[3].z - positionArray[1].z) / 2 * interval;
        return velocityVector;
    }

    Vector3 CalculateAcceleration(Queue<Vector3> positionQueue)
    {
        Vector3[] positionArray = positionQueue.ToArray();
        Vector3 accelerationVector;
        accelerationVector.x = (positionArray[3].x - 2 * positionArray[2].x + positionArray[1].x) / Convert.ToSingle(Math.Pow(interval,2));
        accelerationVector.y = (positionArray[3].y - 2 * positionArray[2].y + positionArray[1].y) / Convert.ToSingle(Math.Pow(interval,2));
        accelerationVector.z = (positionArray[3].z - 2 * positionArray[2].z + positionArray[1].z) / Convert.ToSingle(Math.Pow(interval,2));
        return accelerationVector;
    }

    Vector3 CalculateJerk(Queue<Vector3> positionQueue)
    {
        Vector3[] positionArray = positionQueue.ToArray();
        Vector3 jerkVector;
        jerkVector.x = (positionArray[4].x - 2 * positionArray[3].x + 2 * positionArray[1].x - positionArray[0].x) / Convert.ToSingle(2 * Math.Pow(interval,3));
        jerkVector.y = (positionArray[4].y - 2 * positionArray[3].y + 2 * positionArray[1].y - positionArray[0].y) / Convert.ToSingle(2 * Math.Pow(interval,3));
        jerkVector.z = (positionArray[4].z - 2 * positionArray[3].z + 2 * positionArray[1].z - positionArray[0].z) / Convert.ToSingle(2 * Math.Pow(interval,3));
        return jerkVector;
    }

    float CalculateCurvature(Vector3 velocityVector, Vector3 accelerationVector)
    {
        Vector3 curvatureCross = Vector3.Cross(accelerationVector, velocityVector);
        float curvature = Convert.ToSingle(curvatureCross.magnitude / Math.Pow(velocityVector.magnitude,3));
        return curvature;
    }

    float CalculateTime(Queue<Vector3> accelQueue)
    {
        float time = 0;

        foreach (var accel in accelQueue)
        {
            time += accel.magnitude;
        }

        return time / T;
    }

    Vector3 CalculateSpaceNumerator(Queue<Vector3> positionQueue)
    {
        Vector3[] positionArray = positionQueue.ToArray();
        Vector3 spaceNumerator;
        spaceNumerator.x = Mathf.Abs(positionArray[2].x - positionArray[1].x);
        spaceNumerator.y = Mathf.Abs(positionArray[2].y - positionArray[1].y);
        spaceNumerator.z = Mathf.Abs(positionArray[2].z - positionArray[1].z);
        return spaceNumerator;
    }
    
    float CalculateSpace(Queue<Vector3> spaceNumeratorQueue)
    {
        Vector3[] spaceNumeratorArray = spaceNumeratorQueue.ToArray();
        Vector3 spaceNumeratorSumVector = Vector3.zero;
        Array.ForEach(spaceNumeratorArray, i => spaceNumeratorSumVector += i);

        Vector3 spaceDenominatorVector = Vector3.one;
        float denominatorX = Mathf.Abs(spaceNumeratorArray[0].x - spaceNumeratorArray[T- 1].x);
        float denominatorY = Mathf.Abs(spaceNumeratorArray[0].y - spaceNumeratorArray[T- 1].y);
        float denominatorZ = Mathf.Abs(spaceNumeratorArray[0].z - spaceNumeratorArray[T -1].z);
        if (denominatorX != 0 ) spaceDenominatorVector.x = denominatorX;
        if (denominatorY != 0 ) spaceDenominatorVector.y = denominatorY;
        if (denominatorZ != 0 ) spaceDenominatorVector.z = denominatorZ;

        Vector3 spaceVector = Vector3.zero;
        spaceVector.x = spaceNumeratorSumVector.x / spaceDenominatorVector.x;
        spaceVector.y = spaceNumeratorSumVector.y / spaceDenominatorVector.y;
        spaceVector.z = spaceNumeratorSumVector.z / spaceDenominatorVector.z;
        float space = spaceVector.magnitude;
        return space;
    }

    float CalculateFlow(Queue<Vector3> jerkQueue)
    {
        float flow = 0;

        foreach (var jerk in jerkQueue)
        {
            flow += jerk.magnitude;
        }

        return flow / T;
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
