using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DroneControl : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        rigidBody = GetComponent<Rigidbody>();

        motor1 = transform.Find("motor1");
        motor2 = transform.Find("motor2");
        motor3 = transform.Find("motor3");
        motor4 = transform.Find("motor4");

        audioSources = GetComponentsInChildren<AudioSource>();

        TPVcameraRotation = Camera.main.transform.rotation;
        TPVcameraPosition = Camera.main.transform.position;

        startPosition = transform.position;
        startRotation = transform.rotation;
    }

    // Update is called once per frame
    void Update()
    {
        Camera.main.transform.position = FPV ? transform.position : TPVcameraPosition;
        Camera.main.transform.rotation = FPV ? transform.rotation : TPVcameraRotation;

        if (Input.GetButtonDown("YButton")) // reset
        {
            transform.position = startPosition;
            transform.rotation = startRotation;
            rigidBody.velocity = Vector3.zero;
            rigidBody.angularVelocity = Vector3.zero;
            integralPitch = integralRoll = integralYaw = 0;
            orientation = Vector3.zero;
            desiredPitch = desiredRoll = desiredYaw = 0;
        }
        if (Input.GetButtonDown("AButton"))
            FPV = !FPV;
    }

    Vector3 startPosition;
    Quaternion startRotation;

    public bool FPV = false;

    Quaternion TPVcameraRotation;
    Vector3 TPVcameraPosition;

    Rigidbody rigidBody;

    // 1 2
    // 3 4

    //Vector3 p1 = new Vector3(-0.5f, 0, 0.5f);
    //Vector3 p2 = new Vector3(0.5f, 0, 0.5f);
    //Vector3 p3 = new Vector3(-0.5f, 0, -0.5f);
    //Vector3 p4 = new Vector3(0.5f, 0, -0.5f);

    Transform motor1, motor2, motor3, motor4;

    Vector3 orientation;
    float desiredPitch, desiredYaw, desiredRoll, throttle;
    float integralPitch, integralRoll, integralYaw;

    public float Kpyaw = 0.2f;
    public float Kiyaw = 0.01f;

    AudioSource[] audioSources;

    private void FixedUpdate()
    {
        // sense orientation (gyro)
        var localAngularVelocity = transform.InverseTransformDirection(rigidBody.angularVelocity);

        var dt = Time.fixedDeltaTime;
        orientation += localAngularVelocity * dt;
        float pitch = orientation.x;
        float yaw = orientation.y;
        float roll = orientation.z;


        // control inputs
        var leftHorizontal = Input.GetAxis("Horizontal");
        var leftVertical = Input.GetAxis("Vertical");
        var rightHorizontal = Input.GetAxis("RightHorizontal");
        var rightVertical = Input.GetAxis("RightVertical");


        // anti gravity
        var yLocal = transform.TransformDirection(Vector3.up);
        float hoverThrottle = rigidBody.mass * 9.8f / yLocal.y / 4f;


        throttle = hoverThrottle + (-0.6f * leftVertical);
        if (throttle < 0)
            throttle = 0;

        float s = 0.05f;
        float sYaw = 0.05f;

        desiredYaw += sYaw * leftHorizontal;
        desiredPitch += -s * rightVertical;
        desiredRoll += -s * rightHorizontal;

        // control

        float Kp = 0.2f;
        float Ki = 0.01f;

        // PID controller
        float errorPitch = desiredPitch - pitch - localAngularVelocity.x * dt;
        float proportionalPitch = Kp * errorPitch;
        integralPitch += Ki * errorPitch * dt;
        float dPitch = proportionalPitch + integralPitch;

        float errorRoll = desiredRoll - roll - localAngularVelocity.z * dt;
        float proportionalRoll = Kp * errorRoll;
        integralRoll += Ki * errorRoll * dt;
        float dRoll = proportionalRoll + integralRoll;

        float errorYaw = desiredYaw - yaw - localAngularVelocity.y * dt;
        float proportionalYaw = Kpyaw * errorYaw;
        integralYaw += Kiyaw * errorYaw * dt;
        float dYaw = proportionalYaw + integralYaw;

        //Debug.Log(currentYaw + " " + desiredYaw);


        // TODO:
        // stabilize controller feeds into rate contoller


        // 1 2
        // 3 4


        // motor speeds
        var s1 = throttle - dPitch - dRoll + dYaw;
        var s2 = throttle - dPitch + dRoll - dYaw;
        var s3 = throttle + dPitch - dRoll - dYaw;
        var s4 = throttle + dPitch + dRoll + dYaw;

        s1 = Mathf.Clamp(s1, 0, 0.8f);
        s2 = Mathf.Clamp(s2, 0, 0.8f);
        s3 = Mathf.Clamp(s3, 0, 0.8f);
        s4 = Mathf.Clamp(s4, 0, 0.8f);


        // forces

        var f1 = s1 * Vector3.up;
        var f2 = s2 * Vector3.up;
        var f3 = s3 * Vector3.up;
        var f4 = s4 * Vector3.up;

        var f = f1 + f2 + f3 + f4;

        rigidBody.AddRelativeForce(f);


        // torques

        var p1 = motor1.localPosition;
        var p2 = motor2.localPosition;
        var p3 = motor3.localPosition;
        var p4 = motor4.localPosition;

        var t = Vector3.Cross(p1, f1) + Vector3.Cross(p2, f2) + Vector3.Cross(p3, f3) + Vector3.Cross(p4, f4);
        t += s1 * Vector3.up + s2 * Vector3.down + s3 * Vector3.down + s4 * Vector3.up;

        rigidBody.AddRelativeTorque(t);


        // motor vis
        {
            var scale = motor1.localScale;
            scale.x = scale.z = 2 * s1;
            motor1.localScale = scale;
        }
        {
            var scale = motor2.localScale;
            scale.x = scale.z = 2 * s1;
            motor2.localScale = scale;
        }
        {
            var scale = motor3.localScale;
            scale.x = scale.z = 2 * s1;
            motor3.localScale = scale;
        }
        {
            var scale = motor4.localScale;
            scale.x = scale.z = 2 * s1;
            motor4.localScale = scale;
        }

        // audio

        float b = 0.8f;
        float p = 1f;
        audioSources[0].pitch = b + p * s1;
        audioSources[1].pitch = b + p * s2;
        audioSources[2].pitch = b + p * s3;
        audioSources[3].pitch = b + p * s4;

        audioSources[0].volume = (s1 == 0) ? 0 : 1;
        audioSources[1].volume = (s2 == 0) ? 0 : 1;
        audioSources[2].volume = (s3 == 0) ? 0 : 1;
        audioSources[3].volume = (s4 == 0) ? 0 : 1;

    }

}
