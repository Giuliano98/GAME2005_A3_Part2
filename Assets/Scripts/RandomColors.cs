using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RandomColors : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        GetComponent<Renderer>().material.color = new Color(
            Mathf.Repeat(transform.position.x, 1.0f),
            Mathf.Repeat(transform.position.y, 1.0f), 
            Mathf.Repeat(transform.position.z, 1.0f)
            );
    }
}
