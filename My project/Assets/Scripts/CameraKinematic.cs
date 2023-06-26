using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraKinematic : MonoBehaviour
{
    public RobotJoint[] Joints = new RobotJoint[6];
    // public HingeJoint[] hingeJoints = new HingeJoint[6];
    public float[] Angles = new float[6];

    public Transform trigger;
    public Transform endEffector;


    public float SamplingDistance = 1;
    public float DistanceThreshold = 0.01f;
    public float LearningRate = 100;    

    public float DistanceWeight = 1;
    public float RotationWeight = 1;
    public float TorsionWeight = 1;

    private JointSpring[] spring = new JointSpring[6];
    void Start()
    {
        float[] angles = new float[Joints.Length];

        for (int i = 0; i < Joints.Length; i++)
        {
            if (Joints[i].Axis.x == 1)
            {
                angles[i] = Joints[i].transform.localRotation.eulerAngles.x;
            }
            else if (Joints[i].Axis.y == 1)
            {
                angles[i] = Joints[i].transform.localRotation.eulerAngles.y;
            }
            else if (Joints[i].Axis.z == 1)
            {
                angles[i] = Joints[i].transform.localRotation.eulerAngles.z;
            }
        }
        Angles = angles;

        LearningRate = 50;
    }

    // Update is called once per frame
    void Update()
    {
        InverseKinematics(trigger.position, Angles);
        for (int i = 0; i < Joints.Length; i++)
        {
            if (Joints[i].Axis.x == 1)
            {
                Joints[i].transform.localEulerAngles = new Vector3(-Angles[i], 0, 0);
            }
            else if (Joints[i].Axis.y == 1)
            {
                Joints[i].transform.localEulerAngles = new Vector3(0, Angles[i], 0);
            }
            else if (Joints[i].Axis.z == 1)
            {
                Joints[i].transform.localEulerAngles = new Vector3(0, 0, Angles[i]);
            }
        }

    }

    public Vector3 ForwardKinematics(float[] angles)
    {
        Vector3 prevPoint = Joints[0].transform.position;
        Quaternion rotation = Quaternion.identity;
        for (int i = 1; i < Joints.Length; i++)
        {
            // Rotates around a new axis
            rotation *= Quaternion.AngleAxis(angles[i - 1], Joints[i - 1].Axis);
            Vector3 nextPoint = prevPoint + rotation * Joints[i].StartOffset;

            prevPoint = nextPoint;
        }
        //Debug.Log(prevPoint.y);
        return prevPoint;
    }

    public float DistanceFromTarget(Vector3 target, float[] angles)
    {
        Vector3 point = ForwardKinematics(angles);
        return Vector3.Distance(point, target);
    }

    public float NormalisedDistance(Vector3 target, float[] angles)
    {
        Vector3 point = ForwardKinematics(angles);
        float maxDistance = 2.1f;
        float distance = Vector3.Distance(point, target)/maxDistance;
        return distance;
    }

    public float NormalisedRotation(Vector3 target, float[] angles)
    {
        float rotationPenalty =
        Mathf.Abs
        (
         Quaternion.Angle(endEffector.rotation, trigger.rotation) / 180f
        ) ;
        return rotationPenalty;
    }

    public float NormalisedTorsion(Vector3 target, float[] angles)
    {

        float torsionPenalty = 0;
        for (int i = 0; i< angles.Length; i++)
            torsionPenalty += Mathf.Abs(angles[i]);
        torsionPenalty /= angles.Length;
        return torsionPenalty / 360;
    }
    public float ErrorFunction(Vector3 target, float[] angles)
    {
        Debug.Log(NormalisedRotation(target, angles));


        return
            NormalisedDistance(target, angles) * DistanceWeight +
            NormalisedRotation(target, angles) * RotationWeight +
            NormalisedTorsion(target, angles) * TorsionWeight;
    }



    /*public float PartialGradient(Vector3 target, float[] angles, int i)
    {
        // Saves the angle,
        // it will be restored later
        float angle = angles[i];
        // Gradient : [F(x+SamplingDistance) - F(x)] / h
        float f_x = DistanceFromTarget(target, angles);
        angles[i] += SamplingDistance;
        float f_x_plus_d = DistanceFromTarget(target, angles);
        float gradient = (f_x_plus_d - f_x) / SamplingDistance;
        // Restores
        angles[i] = angle;
        return gradient;
    }*/
    public float PartialGradient(Vector3 target, float[] angles, int i)
    {
        // Saves the angle,
        // it will be restored later
        float angle = angles[i];
        // Gradient : [F(x+SamplingDistance) - F(x)] / h
        float f_x = ErrorFunction(target, angles);
        angles[i] += SamplingDistance;
        float f_x_plus_d = ErrorFunction(target, angles);
        float gradient = (f_x_plus_d - f_x) / SamplingDistance;
        // Restores
        angles[i] = angle;
        return gradient;
    }

    /*public void InverseKinematics(Vector3 target, float[] angles)
    {
        if (DistanceFromTarget(target, angles) < DistanceThreshold)
            return;
        for (int i = Joints.Length - 1; i >= 0; i--)
        {
            // Gradient descent
            // Update : Solution -= LearningRate * Gradient
            float gradient = PartialGradient(target, angles, i);
            angles[i] -= LearningRate * gradient;

            // Clamp
            angles[i] = Mathf.Clamp(angles[i], Joints[i].MinAngle, Joints[i].MaxAngle);
            // Early termination
            if (DistanceFromTarget(target, angles) < DistanceThreshold)
                return;
        }
    }*/
    public void InverseKinematics(Vector3 target, float[] angles)
    {
        if (ErrorFunction(target, angles) < DistanceThreshold)
            return;
        for (int i = Joints.Length - 1; i >= 0; i--)
        {
            // Gradient descent
            // Update : Solution -= LearningRate * Gradient
            float gradient = PartialGradient(target, angles, i);
            angles[i] -= LearningRate * gradient;

            // Clamp
            angles[i] = Mathf.Clamp(angles[i], Joints[i].MinAngle, Joints[i].MaxAngle);
            // Early termination
            if (ErrorFunction(target, angles) < DistanceThreshold)
                return;
        }
    }
}
