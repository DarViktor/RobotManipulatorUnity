using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Valve.VR;
using Valve.VR.InteractionSystem;
public class ColisionDetector : MonoBehaviour
{
    public SteamVR_Action_Boolean klickCloseArm;
    public SteamVR_Action_Boolean klickOpenArm;
    public GameObject armLeft;
    public GameObject armRight;
    public GameObject arm;

    private bool colArm;



    public ReyCast_ armCast;

    private int childrenArms;
    /*private void OnCollisionStay(Collision collision)
    {
        if (armCast.statArm == true)
        {
            if (collision.gameObject)
            {
                if (collision.gameObject.tag == "ArmLeft" *//**//*)
                {
                    colArmLeft = true;
                }
                else if (collision.gameObject.tag == "ArmRight" *//*& childrenArms == 1*//*)
                {
                    colArmRight = true;

                }
                else
                {
                    colArmLeft = false;
                    colArmRight = false;
                }
            }
        }
        else
        {
            colArmLeft = false;
            colArmRight = false;
        }
    }*/

    private void OnTriggerStay(Collider other)
    {
        if (other.gameObject.tag == "Trigger")
        {
            colArm = true;
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.gameObject.tag == "Trigger")
        {
            colArm = false;
        }
    }

    private void Update()
    {
        childrenArms = arm.transform.childCount;
        if (armCast.statArm == false & childrenArms > 2)
        {
            gameObject.GetComponent<Rigidbody>().useGravity = true;
            gameObject.GetComponent<Rigidbody>().constraints = RigidbodyConstraints.None;
            transform.parent = null;
        }
        if (armCast.statArm == true & colArm & childrenArms == 2)
        {
            gameObject.GetComponent<Rigidbody>().constraints = RigidbodyConstraints.FreezeAll;
            gameObject.GetComponent<Rigidbody>().useGravity = false;
            transform.parent = arm.transform;
        }
    }


    private void FixedUpdate()
    {


    }
} 
