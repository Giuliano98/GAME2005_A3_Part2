using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MyLaunchBall : MonoBehaviour
{
    public Transform spawnPos;
    public FizziksObject pingPongBall;
    public FizziksObject baseball;
    public FizziksObject basketball;
    public FizziksObject bowlingBall;

    public int ballSelection = 1;

    void Update()
    {
        if (Input.GetKey (KeyCode.Z))
            ballSelection = 1;
        if (Input.GetKey (KeyCode.X))
            ballSelection = 2;
        if (Input.GetKey (KeyCode.C))
            ballSelection = 3;
        if (Input.GetKey (KeyCode.V))
            ballSelection = 4;

        if (Input.GetButtonDown("Fire1"))
        {
            Vector3 launchVelocity = new Vector3(6,6,6); 

            switch (ballSelection)
            {
                case 1:
                    //pingPongBall.velocity = launchVelocity;
                    Instantiate(pingPongBall, spawnPos);
                    break;
                case 2:
                    //baseball.velocity = launchVelocity;
                    Instantiate(baseball, spawnPos);
                    break;
                case 3:
                   // basketball.velocity = launchVelocity;
                    Instantiate(basketball, spawnPos);
                    break;
                case 4:
                   // bowlingBall.velocity = launchVelocity;
                    Instantiate(bowlingBall, spawnPos);
                    break;
                default:
                    break;
            }
        }

    }
}
