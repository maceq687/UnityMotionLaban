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
    private Camera Cam = Camera.main;
    private float interval = 0.1f; // Sampling interval in seconds
    private int t = 5;
    private float delta = 1; // Time window for time, space and flow efforts calculations in seconds
    private int T;
    private Queue<Vector3> leftHandPositionQueue = new Queue<Vector3>();
    private Queue<Vector3> rightHandPositionQueue = new Queue<Vector3>();
    private Queue<Vector3> headPositionQueue = new Queue<Vector3>();
    private Queue<Vector3> leftHandAccelQueue = new Queue<Vector3>();
    private Queue<Vector3> rightHandAccelQueue = new Queue<Vector3>();
    private Queue<Vector3> headAccelQueue = new Queue<Vector3>();
    private float timeAlpha = 1; // Alpha time for joint(s)


    public void JointTracking()
    {
        leftHand = GameObject.Find("Bot");
        rightHand = GameObject.Find("Bot");
        head = GameObject.Find("Bot");
        T = (int)(delta / interval);
        StartCoroutine(OnFrame());
    }


    IEnumerator OnFrame()
    {
         for(;;) 
        {
            Vector3 leftHandPosition = Cam.WorldToScreenPoint(leftHand.transform.position);
            InsertIntoMemory(leftHandPosition, leftHandPositionQueue);

            Vector3 rightHandPosition = Cam.WorldToScreenPoint(rightHand.transform.position);
            InsertIntoMemory(rightHandPosition, rightHandPositionQueue);

            Vector3 headPosition = Cam.WorldToScreenPoint(head.transform.position);
            InsertIntoMemory(headPosition, headPositionQueue);

            Vector3 leftHandVelocity = CalculateVelocity(leftHandPositionQueue);
            Vector3 rightHandVelocity = CalculateVelocity(rightHandPositionQueue);
            Vector3 headVelocity = CalculateVelocity(headPositionQueue);

            Vector3 leftHandAccel = CalculateAcceleration(leftHandPositionQueue);
            InsertIntoMemory(leftHandAccel, leftHandAccelQueue);

            Vector3 rightHandAccel = CalculateAcceleration(rightHandPositionQueue);
            InsertIntoMemory(rightHandAccel, rightHandAccelQueue);

            Vector3 headAccel = CalculateAcceleration(headPositionQueue);
            InsertIntoMemory(headAccel, headAccelQueue);

            float leftHandTime = CalculateTime(leftHandAccelQueue);
            float rightHandTime = CalculateTime(rightHandAccelQueue);
            float headTime = CalculateTime(headAccelQueue);

            float time = timeAlpha * leftHandTime + timeAlpha * rightHandTime - timeAlpha * headTime;

            yield return new WaitForSeconds(interval);
        }
    }

    void InsertIntoMemory<Type>(Type elem, Queue<Type> queue)
    {
        if (queue.Count == t) queue.Dequeue();
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
}
