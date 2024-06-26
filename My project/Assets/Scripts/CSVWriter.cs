using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class CSVWriter : MonoBehaviour
{
    string filname = "";

    public Transform arm1;
    public Transform target1;

    public Transform arm2;
    public Transform camera;



    bool startSymb = true;
    bool endSymb = false;

    int numFile = 1;

    // Start is called before the first frame update
    void Start()
    {
        filname = Application.dataPath + "/test" + numFile + ".csv";
    }
    TextWriter tw;

    // Update is called once per frame
    void FixedUpdate()
    {
        /*        if (Input.GetKeyDown(KeyCode.Space))
                {
                    WriteCSW();
                }*/
        if(endSymb == false)
        {
            
            if (startSymb == true)
            {
                tw = new StreamWriter(filname, false);
                tw.WriteLine("Arm1;   ;   ; target;   ;   ; Arm2;   ;   ; camera");
                tw.WriteLine("  x1; y1; z1;     x2; y2; z2;   x3; y3; z3; x4; y4; z4;");
                startSymb = false;
                Debug.Log("Start write CSW");
            }
            tw.WriteLine(arm1.position.x + ";" + arm1.position.y + ";" + arm1.position.z +
                ";" + target1.position.x + ";" + target1.position.y + ";" + target1.position.z +
                ";" + arm2.position.x + ";" + arm2.position.y + ";" + arm2.position.z +
                ";" + camera.rotation.x + ";" + camera.rotation.y + ";" + camera.rotation.z);

            Debug.Log("Write new line");

            if (Input.GetKeyDown(KeyCode.Space))
            {
                endSymb = true;
            }
            
            if (endSymb == true)
            {
                tw.Dispose();
                Debug.Log("End write CSW");
            }
        }

    }

    public void StopWriteCSW()
    {

    }

/*    public void WriteCSW(TextWriter tw)
    {

    }*/

    
}
