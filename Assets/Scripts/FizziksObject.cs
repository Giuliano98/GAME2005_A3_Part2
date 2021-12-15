using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FizziksObject : MonoBehaviour
{
    public float mass = 1.0f;               // 1Kg == 1.0f
    public Vector3 velocity = Vector3.zero; // m/s
    public float gravityScale = 1.0f;       // m^2/s
    public float bounciness = 0.6f;         // [0, 1]
    public float frictionless = 0.5f;       // [0,1]

    //If true, this object will not be moved by our Fizziks system
    public bool lockPosition = false;
    
    public FizziksColliderBase shape = null; 
    
    void Start()
    {
        // fill my object list
        FindObjectOfType<FizziksSystem>().fizziksObjects.Add(this);
        shape = GetComponent<FizziksColliderBase>();
    }
} 
