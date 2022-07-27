using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Projectile : MonoBehaviour
{
    public Vector3 InitialForce;
    public float Delay = 0.0f;
    private float TimePassed = 0.0f;
    private bool Fired = false;
    // Start is called before the first frame update
    void Start()
    {
        GetComponent<Rigidbody>().useGravity = false;
    }

    // Update is called once per frame
    void Update()
    {
        if (!Fired)
        {
            TimePassed += Time.deltaTime;
            if (TimePassed > Delay)
            {
                Rigidbody rigidbody = GetComponent<Rigidbody>();
                if (rigidbody != null)
                {
                    GetComponent<Rigidbody>().useGravity = true;
                    rigidbody.AddForce(InitialForce);
                    Fired = true;
                }
            }
        }
    }
}
