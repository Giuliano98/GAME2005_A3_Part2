using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public enum Axis
{
    X = 0,
    Y,
    Z
}

//We can define a plane with a normal vector, and a point on the plane
public class FizziksColliderPlane : FizziksColliderBase
{
    public Axis alignment = Axis.Y;
    
    public override CollisionShape GetCollisionShape()
    {
        return CollisionShape.Plane;
    }

    public Vector3 GetNormal()
    {
        switch (alignment)
        {
            case(Axis.X):
            {
                return transform.right;
            }
            case(Axis.Y):
            {
                return transform.up;
            }
            case(Axis.Z):
            {
                return transform.forward;
            }
            default:
            {
                throw new System.Exception("Invalid plane alignment????");
            }
        }
    }
}
