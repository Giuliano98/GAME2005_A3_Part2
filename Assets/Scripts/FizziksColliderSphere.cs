using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FizziksColliderSphere : FizziksColliderBase
{
    public float radius = 1.0f;
    public override CollisionShape GetCollisionShape()
    {
        return CollisionShape.Sphere;
    }
}
