using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Valve.VR;
using Valve.VR.InteractionSystem;

public class ReyCast_ : MonoBehaviour
{
    public SteamVR_Input_Sources hand;
    public Transform Pointer;

    public SteamVR_Action_Boolean klickGrab;

    public SteamVR_Action_Boolean klickCloseArm;
    public SteamVR_Action_Boolean klickOpenArm;
    public Vector3 posArmLeft;
    public Vector3 posArmRight;
    public Quaternion orintArmLeft;
    public Quaternion orintArmRight;
    public Transform armLeft, armRight;
/*    public ConfigurableJoint armLeft, armRight;
*/    private float speedCloseArm = 0.3f;
    public bool statArm = false;

    public SteamVR_Action_Vector2 klickMoveForward;

    private GameObject target;
    //public SteamVR_Action_Vector2 klickMoveBack;
    private Vector3 leftHomePoint = new Vector3(-1.34f, 0, -0.4f);
    private Vector3 rightHomePoint = new Vector3(-1.34f, 0, 0.4f);
    private Vector3 leftTargetPoint = new Vector3(-1.34f, 0, -0.4f);
    private Vector3 rightTargetPoint = new Vector3(-1.34f, 0, 0.4f);    


    [SerializeField] private LayerMask _targetLayerMask;
    private bool grabed = false;

    void Update()
    {
        Vector3 vec = transform.forward;
        Ray ray = new Ray(transform.position, vec * 100);
        /*Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);*/
        Debug.DrawRay(transform.position, vec * 100, Color.yellow);

        RaycastHit hit;
        if (Physics.Raycast(ray, out hit,100f,_targetLayerMask))
        {
            Pointer.position = hit.point;
        }
        if (hit.collider.gameObject.tag == "Target")
        {
            if (klickGrab[hand].stateDown)
            {
                grabed = true;
                target = hit.collider.gameObject;
                hit.transform.parent = gameObject.transform;
            }

            

        }
        if (klickGrab[hand].stateUp)
        {
            grabed = false;
            target.transform.parent = null;
        }
        if (grabed & hit.collider.gameObject.GetComponent<Selectable>())
        {
            hit.transform.localPosition = hit.transform.localPosition + new Vector3(0, 0, klickMoveForward.axis.y * 0.1f);
        }
        
        if (klickCloseArm.stateDown)
        {
            //armLeft.transform.GetLocalPositionAndRotation(out posArmLeft, out orintArmLeft);
            if (!statArm)
            {
                leftTargetPoint.z = -speedCloseArm;
                rightTargetPoint.z = speedCloseArm;

                /*armLeft.targetPosition = armLeft.targetPosition - new Vector3(speedCloseArm, 0, 0);
                armRight.targetPosition = armRight.targetPosition + new Vector3(speedCloseArm, 0, 0);*/

                statArm = true;
            }
        }
        if (klickOpenArm.stateDown)
        {
            if (statArm)
            {
                leftTargetPoint.z = leftHomePoint.z;
                rightTargetPoint.z = rightHomePoint.z;
                /*armLeft.targetPosition = armLeft.targetPosition + new Vector3(speedCloseArm, 0, 0);
                armRight.targetPosition = armRight.targetPosition - new Vector3(speedCloseArm, 0, 0);*/
                statArm = false;
            }
        }
        Vector3 errLeft =  new Vector3(0, 0, (armLeft.localPosition.z - leftTargetPoint.z));
        Vector3 errRight = new Vector3(0, 0, (armRight.localPosition.z - rightTargetPoint.z));

        armLeft.localPosition = armLeft.localPosition - errLeft * Time.deltaTime;
        armRight.localPosition = armRight.localPosition - errRight * Time.deltaTime;

    }

    private void FixedUpdate()
    {

    }
}
