using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
public enum CollisionShape
{
    Sphere = 0,
    Plane,
    AABB
}

struct CollisionInfo
{
    public FizziksColliderBase objectA;
    public FizziksColliderBase objectB;
    public Vector3 collisionNormalAtoB;
    public Vector3 contactPoint;
}

//Unity-specific Attribute which says this component needs another component on a gameobject to work
[RequireComponent(typeof(FizziksObject))]
//For a script in Unity, the Monobehavior child class name has to have the same name as the source .cs file
public abstract class FizziksColliderBase : MonoBehaviour
{
    //abstract means that this must be overridden in a child class or else there will be an error
    //abstract is the same as pure virtual in c++
    public abstract CollisionShape GetCollisionShape();

    public FizziksObject kinematicsObject;

    public void Start()
    {
        kinematicsObject = GetComponent<FizziksObject>();
    }
}
