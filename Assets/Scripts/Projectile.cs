using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Projectile : MonoBehaviour
{
    public Vector3 InitialForce;
    // Start is called before the first frame update
    void Start()
    {
        Rigidbody rigidbody = GetComponent<Rigidbody>();
        if (rigidbody != null)
		{
            rigidbody.AddForce(InitialForce);
		}
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
