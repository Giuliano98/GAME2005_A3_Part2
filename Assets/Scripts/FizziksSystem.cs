using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FizziksSystem : MonoBehaviour
{
    public Vector3 gravity = new Vector3(0, -9.81f, 0);
    public List<FizziksObject> fizziksObjects = new List<FizziksObject>();

    //Smallest distance before things get FUNKY
    public float minimumDistance = 0.0001f;

    void FixedUpdate()
    {
        //Velocity update
        for (int i = 0; i < fizziksObjects.Count; i++)
        {
            FizziksObject obj = fizziksObjects[i];
            if (!obj.lockPosition)
                obj.velocity += (gravity * obj.gravityScale) * Time.fixedDeltaTime;
        }

        //Position update
        for (int i = 0; i < fizziksObjects.Count; i++)
        {
            FizziksObject obj = fizziksObjects[i];
            if (!obj.lockPosition)
                obj.transform.position += (obj.velocity * Time.fixedDeltaTime);
        }

        // Update all my objects collisions
        CollisionUpdate();
    }

    void CollisionUpdate()
    {
        // Go trough all the obj I have
        for (int objectIndexA = 0; objectIndexA < fizziksObjects.Count; objectIndexA++)
        {
            for (int objectIndexB = objectIndexA + 1; objectIndexB < fizziksObjects.Count; objectIndexB++)
            {
                FizziksObject objectA = fizziksObjects[objectIndexA];
                FizziksObject objectB = fizziksObjects[objectIndexB];

                //If one does not have a collider...
                if (objectA.shape == null || objectB.shape == null)
                    continue;

                //If both are spheres...
                //GetCollisionShape is defined in the base class to allow us to determine what derived classes to cast to
                // Sphere-Sphere Collision
                if (objectA.shape.GetCollisionShape() == CollisionShape.Sphere &&
                    objectB.shape.GetCollisionShape() == CollisionShape.Sphere)
                {
                    SphereSphereCollision((FizziksColliderSphere)objectA.shape, (FizziksColliderSphere)objectB.shape);
                }
                // -----------------------

                // Sphere-Plan Collision
                if (objectA.shape.GetCollisionShape() == CollisionShape.Sphere &&
                    objectB.shape.GetCollisionShape() == CollisionShape.Plane)
                {
                    SpherePlaneCollision((FizziksColliderSphere)objectA.shape, (FizziksColliderPlane)objectB.shape);
                }

                if (objectA.shape.GetCollisionShape() == CollisionShape.Plane &&
                    objectB.shape.GetCollisionShape() == CollisionShape.Sphere)
                {
                    SpherePlaneCollision((FizziksColliderSphere)objectB.shape, (FizziksColliderPlane)objectA.shape);
                }
                // ---------------------

                // AABB-AABB Collision
                if (objectA.shape.GetCollisionShape() == CollisionShape.AABB &&
                    objectB.shape.GetCollisionShape() == CollisionShape.AABB)
                {
                    AABBAABBCollision((FizziksColliderAABB)objectA.shape, (FizziksColliderAABB)objectB.shape);
                }
                // -------------------

                // Sphere-AABB Collision
                if (objectA.shape.GetCollisionShape() == CollisionShape.Sphere &&
                    objectB.shape.GetCollisionShape() == CollisionShape.AABB)
                {
                    SphereAABBCollision((FizziksColliderSphere)objectA.shape, (FizziksColliderAABB)objectB.shape);
                }

                if (objectA.shape.GetCollisionShape() == CollisionShape.AABB &&
                    objectB.shape.GetCollisionShape() == CollisionShape.Sphere)
                {
                    SphereAABBCollision((FizziksColliderSphere)objectB.shape, (FizziksColliderAABB)objectA.shape);
                }
                // ---------------------                
            }
        }
    }

    
    // gets the scalars of movements from 2 objects
    void GetLockedMovementScalars(FizziksObject a, FizziksObject b, out float movementScalarA, out float movementScalarB)
    {
        //If A is locked and B is not
        // A*0
        // B*1
        if (a.lockPosition && !b.lockPosition)
        {
            movementScalarA = 0.0f;
            movementScalarB = 1.0f;
            return;
        }

        //If B is locked and A is not
        // A*1
        // B*0
        if (!a.lockPosition && b.lockPosition)
        {
            movementScalarA = 1.0f;
            movementScalarB = 0.0f;
            return;
        }

        //If neither are locked
        // A*0.5
        // B*0.5
        if (!a.lockPosition && !b.lockPosition)
        {
            movementScalarA = 0.5f;
            movementScalarB = 0.5f;
            return;
        }

        //If both are locked
        // A*0.0
        // B*0.0
        movementScalarA = 0.0f;
        movementScalarB = 0.0f;
    }

    void SphereSphereCollision(FizziksColliderSphere a, FizziksColliderSphere b)
    { 
        // If the distance between spheres is less than the sum of their radii, then they are overlapping
        Vector3 displacment = b.transform.position - a.transform.position;
        float distance = displacment.magnitude;
        float sumRadii = a.radius + b.radius;
        float penetrationDepth = sumRadii - distance;
        bool isOverlapping = penetrationDepth > 0;

        if (!isOverlapping)
            return;
        

        Vector3 collisionNormalAtoB;
        if (distance < minimumDistance)
        {
            collisionNormalAtoB = new Vector3(0, penetrationDepth, 0);
        }
        else
        {
            collisionNormalAtoB = displacment / distance;
        }

        //Our minimum translation vector is the vector we have to move along so the objects no longer overlap
        Vector3 minimumTranslationVectorAtoB = penetrationDepth * collisionNormalAtoB;
        Vector3 contactPoint = a.transform.position + collisionNormalAtoB * a.radius;

        // Apply a momento to the objects so it doesnt overlapping
        ApplyMinimumTranslationVector(a, b, minimumTranslationVectorAtoB, collisionNormalAtoB, contactPoint);
    }

    void SpherePlaneCollision(FizziksColliderSphere a, FizziksColliderPlane b)
    {
        Vector3 somePointOnThePlane = b.transform.position;
        Vector3 centerOfSphere = a.transform.position;

        //Construct any vector from the plane to the center of the sphere
        Vector3 fromPlaneToSphere = centerOfSphere - somePointOnThePlane;

        //Use dot product to find the length of the projection of the center of the sphere sphere onto the plane normal
        //This gives the shortest distance from the plane to the center of the sphere.
        //The sign of this dot product indicates which side of the normal this fromPlaneToSphere vector is on.
        //If the sign is negative, they point in opposite directions
        //If the sign is positive, they are at least somewhat in the same direction
        float dot = Vector3.Dot(fromPlaneToSphere, b.GetNormal());
       
        float distance = dot; // Abs(dot) will do plane collision.

        //If the distance is less than the radius of the sphere, they are overlapping
        float penetrationDepth = a.radius - distance;
        bool isOverlapping = penetrationDepth > 0;

        if (!isOverlapping)
            return;

        Vector3 normal = -b.GetNormal();
        Vector3 mtv = normal * penetrationDepth;
        Vector3 contact = centerOfSphere + (dot * normal);
        

        ApplyMinimumTranslationVector(a, b, mtv, normal, contact);
    }

    // add AABBAABBCollision
    void AABBAABBCollision(FizziksColliderAABB objectA, FizziksColliderAABB objectB)
    {
        Vector3 halfSizeA = objectA.GetHalfSize();
        Vector3 halfSizeB = objectB.GetHalfSize();

        Vector3 displacementAtoB = objectB.transform.position - objectA.transform.position;
        float distX = Mathf.Abs(displacementAtoB.x);
        float distY = Mathf.Abs(displacementAtoB.y);
        float distZ = Mathf.Abs(displacementAtoB.z);

        float penetrationX = halfSizeA.x + halfSizeB.x - distX;
        float penetrationY = halfSizeA.y + halfSizeB.y - distY;
        float penetrationZ = halfSizeA.z + halfSizeB.z - distZ;

        if (penetrationX < 0 || penetrationY < 0 || penetrationZ < 0)
            return;


        Vector3 minimunTranslationVectorAtoB;
        Vector3 collisionNormalAtoB;
        Vector3 contac;
        if (penetrationX < penetrationY && penetrationX < penetrationZ)
        {
            collisionNormalAtoB = new Vector3(Mathf.Sign(displacementAtoB.x), 0, 0);
            minimunTranslationVectorAtoB = collisionNormalAtoB * penetrationX;
        }
        else if (penetrationY < penetrationX && penetrationY < penetrationZ)
        {
            collisionNormalAtoB = new Vector3(0, Mathf.Sign(displacementAtoB.y), 0);
            minimunTranslationVectorAtoB = collisionNormalAtoB * penetrationY;
        }
        else
        {
            collisionNormalAtoB = new Vector3(0, 0, Mathf.Sign(displacementAtoB.z));
            minimunTranslationVectorAtoB = collisionNormalAtoB * penetrationZ;
        }

        contac = objectA.transform.position + minimunTranslationVectorAtoB;
        ApplyMinimumTranslationVector(objectA, objectB, minimunTranslationVectorAtoB, collisionNormalAtoB, contac);
    }

    // add SphereAABBCollision
    void SphereAABBCollision(FizziksColliderSphere objectA, FizziksColliderAABB objectB)
    {
        Vector3 centerOfSphere = objectA.transform.position;
        Vector3 halfSizeAABB = objectB.GetHalfSize();

        Vector3 displacementSphereToAABB = objectB.transform.position - objectA.transform.position;
        float distX = Mathf.Abs(displacementSphereToAABB.x);
        float distY = Mathf.Abs(displacementSphereToAABB.y);
        float distZ = Mathf.Abs(displacementSphereToAABB.z);

        float penetrationX = objectA.radius + halfSizeAABB.x - distX;
        float penetrationY = objectA.radius + halfSizeAABB.y - distY;
        float penetrationZ = objectA.radius + halfSizeAABB.z - distZ;


        if (penetrationX < 0 || penetrationY < 0 || penetrationZ < 0)
            return;
        

        Vector3 minimunTranslationVectorAtoB;
        Vector3 collisionNormalSphereToAABB;
        Vector3 contac;

        if (penetrationX < penetrationY && penetrationX < penetrationZ)
        {
            collisionNormalSphereToAABB = new Vector3(Mathf.Sign(displacementSphereToAABB.x), 0, 0);
            minimunTranslationVectorAtoB = collisionNormalSphereToAABB * penetrationX;
        }
        else if (penetrationY < penetrationX && penetrationY < penetrationZ)
        {
            collisionNormalSphereToAABB = new Vector3(0, Mathf.Sign(displacementSphereToAABB.y), 0);
            minimunTranslationVectorAtoB = collisionNormalSphereToAABB * penetrationY;
        }
        else
        {
            collisionNormalSphereToAABB = new Vector3(0, 0, Mathf.Sign(displacementSphereToAABB.z));
            minimunTranslationVectorAtoB = collisionNormalSphereToAABB * penetrationZ;
        }

        contac = objectA.transform.position + minimunTranslationVectorAtoB;
        ApplyMinimumTranslationVector(objectA, objectB, minimunTranslationVectorAtoB, collisionNormalSphereToAABB, contac);
    }
    
    // Apply a Bounce effect to the objects
    void ApplyVelocityResponse(CollisionInfo collisionInfo)
    {
        FizziksObject objA = collisionInfo.objectA.kinematicsObject;
        FizziksObject objB = collisionInfo.objectB.kinematicsObject;
        Vector3 normal = collisionInfo.collisionNormalAtoB;

        Vector3 relativeVelocityAB = objB.velocity - objA.velocity;
        float relativeNormalVelocityAB = Vector3.Dot(relativeVelocityAB, normal);

        // No bounce - check
        if (relativeNormalVelocityAB >= 0.0f)
            return;

        float restitution = (objA.bounciness + objB.bounciness) * 0.5f;


        float deltaV = -(relativeNormalVelocityAB * (1.0f + restitution));
        float impulse;

        if (!objA.lockPosition && objB.lockPosition)
        {
            impulse = deltaV * objA.mass;
            objA.velocity -= normal * impulse / objA.mass;

        }
        else if (objA.lockPosition && !objB.lockPosition)
        {
            impulse = deltaV * objB.mass;
            objB.velocity += normal * impulse / objB.mass;
        }
        else if (!objA.lockPosition && !objB.lockPosition)
        {
            impulse = deltaV / ((1.0f / objA.mass) + (1.0f / objB.mass));
            objA.velocity -= normal * impulse / objA.mass;
            objB.velocity += normal * impulse / objB.mass;
        }
        else
        {
            return;
        }

        Vector3 relativeSurfaceVelocity = relativeVelocityAB - (relativeNormalVelocityAB * normal);

        ApplyFriction(objA, objB, relativeSurfaceVelocity, normal);
    }

    // Compute the Frictions of 2 objects
    void ApplyFriction(FizziksObject a, FizziksObject b, Vector3 relativeSurfaceVelocityAB, Vector3 normalAtoB)
    {
        float minFrictionSpeed = 0.0001f;
        float relativeSpeed = relativeSurfaceVelocityAB.magnitude;

        if (relativeSpeed < minFrictionSpeed)
            return;

        float kFrictionCoefficient = (a.frictionless + b.frictionless) / 2.0f;
        Vector3 directionToApplyFriction = relativeSurfaceVelocityAB / relativeSpeed;
        float gravityAccelerationAlongNormal = Vector3.Dot(gravity, normalAtoB);

        Vector3 frictionAcceleration = directionToApplyFriction * gravityAccelerationAlongNormal * kFrictionCoefficient;

        if (!a.lockPosition)
        {
            a.velocity += frictionAcceleration * Time.fixedDeltaTime;
        }
        if (!b.lockPosition)
        {
            b.velocity += frictionAcceleration * Time.fixedDeltaTime;
        }

    }
    // move the objects apart
    private void ApplyMinimumTranslationVector(FizziksColliderBase a, FizziksColliderBase b, Vector3 minimumTranslationVectorAtoB, Vector3 normal, Vector3 contactPoint)
    {
        GetLockedMovementScalars(a.kinematicsObject, b.kinematicsObject, out float movementScalarA, out float movementScalarB);

        Vector3 translationVectorA = -minimumTranslationVectorAtoB * movementScalarA;
        Vector3 translationVectorB = minimumTranslationVectorAtoB * movementScalarB;

        a.transform.position += translationVectorA;
        b.transform.position += translationVectorB;

        CollisionInfo collisionInfo;
        collisionInfo.objectA = a;
        collisionInfo.objectB = b;
        collisionInfo.collisionNormalAtoB = normal;
        collisionInfo.contactPoint = contactPoint;

        // Bounce?
        ApplyVelocityResponse(collisionInfo);
    }

}