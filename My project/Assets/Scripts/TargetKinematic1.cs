using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TargetKinematic1: MonoBehaviour
{
    public RobotJoint[] Joints = new RobotJoint[7];
    // public HingeJoint[] hingeJoints = new HingeJoint[6];
    public float[] Angles = new float[7];

    public Transform trigger;

    public float SamplingDistance = 1;
    public float DistanceThreshold = 0.01f;
    public float LearningRate = 100;

    //private JointSpring[] spring = new JointSpring[6];
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
        
        LearningRate = 70;
    }

    // Update is called once per frame
    void Update()
    {
        //Debug.Log(Angles[5]);
        InverseKinematics(trigger.position, Angles);
        for (int i = 0; i < Joints.Length; i++)
        {
            if (Joints[i].Axis.x == 1)
            {
                Joints[i].transform.localEulerAngles = new Vector3(Angles[i], 0, 0);
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


        //Vector3 prevRotation = 
        Quaternion rotation = Quaternion.identity;
        for (int i = 1; i < Joints.Length; i++)
        {
            // Rotates around a new axis
            rotation *= Quaternion.AngleAxis(angles[i - 1], Joints[i - 1].Axis);
            Vector3 nextPoint = prevPoint + rotation * Joints[i].StartOffset;
          //  Vector3 nextRotation = prevRotation + rotation;
            prevPoint = nextPoint;
        }
/*        Debug.Log("-----------------------------------------");
        Debug.Log(prevPoint);
        Debug.Log("-----------------------------------------");*/

        return prevPoint;
    }
    public Vector3 ForwardRotationKinematics(float[] angles)
    {
        Quaternion rotation = Quaternion.identity;
        for (int i = 1; i < Joints.Length; i++)
        {
            // Rotates around a new axis
            rotation *= Quaternion.AngleAxis(angles[i - 1], Joints[i - 1].Axis);          
        }
        //Debug.Log(rotation.eulerAngles);
        return rotation.eulerAngles;
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
        float distance = Vector3.Distance(point, target) / maxDistance;
        return distance;
    }

    public float NormalisedRotation(Vector3 target, float[] angles)
    {
        Vector3 rPosOrient = ForwardRotationKinematics(angles);
        float rotationPenalty =
        Mathf.Abs
        (
         Quaternion.Angle(Quaternion.Euler(new Vector3(rPosOrient.x, rPosOrient.y, rPosOrient.z)), Quaternion.Euler(new Vector3(-trigger.eulerAngles.x, trigger.eulerAngles.y, trigger.eulerAngles.z))) / 360f
        );
        return rotationPenalty;
    }

    public float NormalisedTorsion(Vector3 target, float[] angles)
    {

        float torsionPenalty = 0;
        for (int i = 0; i < angles.Length; i++)
            torsionPenalty += Mathf.Abs(angles[i]);
        torsionPenalty /= angles.Length;
        return torsionPenalty / 360;
    }
    public float ErrorFunction(Vector3 target, float[] angles)
    {
        //Debug.Log(NormalisedDistance(target, angles));
        float distanceWeight = 0;
        float rotationWeight = 0.8f;
        float torsionWeight = 0;
        return
            NormalisedDistance(target, angles) * distanceWeight +
            NormalisedRotation(target, angles) * rotationWeight +
            NormalisedTorsion(target, angles) * torsionWeight;
    }



    public float PartialGradient(Vector3 target, float[] angles, int i)
    {
        // Saves the angle,
        // it will be restored later
        float angle = angles[i];
        // Gradient : [F(x+SamplingDistance) - F(x)] / h
        float f_x = ErrorFunction(target, angles);
        angles[i] += SamplingDistance;
        //if (i == 5) Debug.Log(angles[i]);
        float f_x_plus_d = ErrorFunction(target, angles);
        float gradient = (f_x_plus_d - f_x) / SamplingDistance;
        // Restores
        angles[i] = angle;
        return gradient;
    }

    public void InverseKinematics(Vector3 target, float[] angles)
    {
        Vector3 targetPos = target;
        //targetPos.x = -targetPos.x;
        //targetPos.y = -targetPos.y;
        //targetPos.z = targetPos.z + (float)5.156 + (float)2.4;

        if (ErrorFunction(target, angles) < DistanceThreshold)
            return;
        for (int i = Joints.Length-1; i >= 0; i--)
        {
            // Gradient descent
            // Update : Solution -= LearningRate * Gradient
            float gradient = PartialGradient(targetPos, angles, i);
            angles[i] -= LearningRate * gradient;
            //if (i == 5) Debug.Log(gradient);
            // Clamp
            angles[i] = Mathf.Clamp(angles[i], Joints[i].MinAngle, Joints[i].MaxAngle);
            // Early termination
            //Debug.Log(ErrorFunction(targetPos, angles));
            if (ErrorFunction(targetPos, angles) < DistanceThreshold)
            return;
        }
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
    }

    public void InverseKinematics(Vector3 target, float[] angles)
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
}
