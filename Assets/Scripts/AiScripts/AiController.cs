﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace UnityStandardAssets.Vehicles.MyCar {

	public class AiController : MonoBehaviour {

		public Transform path;
		private List<Transform> wayPoints = new List<Transform> ();
		int _myWaypointIndex = 0;		// used as index for My_Waypoints
		private Rigidbody m_Rigidbody;

		//public GameObject[] myWaypoints; 

		//[Range(0.0f, 300.0f)] // create a slider in the editor and set limits on moveSpeed
		//[SerializeField] private float moveSpeed = 80f; // enemy move speed
		[SerializeField] float maxSpeed = 300f;
		[SerializeField] private float m_Downforce = 100f;
		private float currentSpeed;
		[SerializeField] private SpeedType m_SpeedType;
		//[Range(0.0f, 200.0f)] 
		//public float rotationSpeed = 100.0f;
		public float maxSteerAngel = 45f;
		public float turnSpeed = 5f;
		private float m_CurrentTorque;


		[SerializeField] private WheelCollider[] m_WheelColliders = new WheelCollider[4];
		[SerializeField] private GameObject[] m_WheelMeshes = new GameObject[4];
		[SerializeField] private WheelEffects[] m_WheelEffects = new WheelEffects[4];
		private Quaternion[] m_WheelMeshLocalRotations;

        [SerializeField] private CarDriveType m_CarDriveType;

        [Header("Torque")]
		[SerializeField] private float m_FullTorqueOverAllWheels;
		[SerializeField] private float m_ReverseTorque;
		[SerializeField] private float m_BrakeTorque;
		[SerializeField] private float m_SlipLimit;
		[SerializeField] private Vector3 m_CentreOfMassOffset;

		public Light[] lights;
		public Light[] brakeLights;
		public KeyCode keyboard;
		//public KeyCode keyboard2;

		public bool isBraking = false;
		private float maxBrakeTorque = 350f;
		//public float CurrentSpeed{ get { return gameObject.GetComponent<Rigidbody>().velocity.magnitude*2.23693629f; }}
		private float CurrentSpeed;

		//Gearbox
		[Header("GearBox")]
		[SerializeField] private static int NoOfGears = 7;
		private int m_GearNum = 0;
		private float m_GearFactor;
		[SerializeField] private float m_RevRangeBoundary = 1f;

		//Acceleration
		public float Revs { get; set; }
		[Range(0.0f, 5.0f)]
		public float AccelInput;
		public float BrakeInput;
        [SerializeField] private int torqueSteeringDiscount = 100;
        [SerializeField] private float maxSteeringAngelOnFullSpeed = 3f;

        //Sensors
        [Header("Sensors")]
		public float sensorLenght = 3f;
		public Vector3 frontSensorPosition = new Vector3(0, 0.5f, 0.5f);
		public float frontSideSensorPos = 1.5f; 
		public float frontSensorAngel = 30f;
        public Vector3 backSensorPos = new Vector3(0, 0.5f, -0.5f);
        public float rearSideSensorPos = 1.5f;
        public float rearSensorAngel = -30f;
        private bool avoiding = false;

        private float targetSteerAngel = 0f;

        private bool isCollided = false;
        private bool isCornering = false;
        private bool isCollidedOnReverse = false;

        [Header("Respawners")]
        [SerializeField] private Transform respawner;

        [SerializeField] private AudioClip checkpointSFX;

		// Use this for initialization
		void Start () {
			wayPoints = getPathTransform ();

			m_WheelMeshLocalRotations = new Quaternion[4];
			for (int i = 0; i < 4; i++)
			{
				m_WheelMeshLocalRotations[i] = m_WheelMeshes[i].transform.localRotation;
			}
			m_WheelColliders[0].attachedRigidbody.centerOfMass = m_CentreOfMassOffset;

			m_Rigidbody = transform.GetComponent<Rigidbody> ();

			m_CurrentTorque = m_FullTorqueOverAllWheels; // - (m_TractionControl*m_FullTorqueOverAllWheels);
		}
			
		void FixedUpdate () {

			CurrentSpeed = transform.GetComponent<Rigidbody>().velocity.magnitude*2.23693629f;

            //print (CurrentSpeed);

            if(CurrentSpeed >= 5f && isCollided)
            {
                print("RESET");
                StopReverse();
                isCollided = false;
            }

            Sensors ();

            //print(targetSteerAngel + " ");

            if (Mathf.Abs(targetSteerAngel) > maxSteeringAngelOnFullSpeed && CurrentSpeed > 10f)
            {
                //print("Steering...");
                isCornering = true;
            }
            else
            {
                isCornering = false;
            }

            if (!isCollided)
            {
                ApplySteer();
                Movement();
                CheckWaypointDistance();
                //AiCarBraking ();
                LerpToSteerAngel();
            }
		}

		void Sensors() {
			RaycastHit hit;
			Vector3 sensorStartingPosition = transform.position;// + frontSensorPosition;
			sensorStartingPosition += transform.forward * frontSensorPosition.z;
			sensorStartingPosition += transform.up * frontSensorPosition.y;

            Vector3 backSensorStartingPos = transform.position;
            backSensorStartingPos -= transform.forward * backSensorPos.z;
            backSensorStartingPos += transform.up * backSensorPos.y;

            float avoidMultiplier = 0;
			avoiding = false;

			//Front Sensor
			if (Physics.Raycast (sensorStartingPosition, transform.forward, out hit, sensorLenght)) {
				if (!hit.collider.CompareTag ("Terrain") && !hit.collider.CompareTag ("Ai")) {
					//print (hit.collider.tag);
					Debug.DrawLine (sensorStartingPosition, hit.point);
					avoiding = true;
                    isCollidedOnReverse = false;
                }
			}


			//Front Right Sensor
			sensorStartingPosition += transform.right * frontSideSensorPos;
			if (Physics.Raycast (sensorStartingPosition, transform.forward, out hit, sensorLenght)) {
				if (!hit.collider.CompareTag ("Terrain") && !hit.collider.CompareTag ("Ai")) {
					//print (hit.collider.tag + "..." + hit.collider.name);
					Debug.DrawLine (sensorStartingPosition, hit.point);
					avoiding = true;
					avoidMultiplier -= 1f;
                    isCollidedOnReverse = false;
                }
			} //Side Right Sensor
			else if (Physics.Raycast (sensorStartingPosition, Quaternion.AngleAxis(frontSensorAngel, transform.up) * transform.forward, out hit, sensorLenght)) {
				if (!hit.collider.CompareTag ("Terrain") && !hit.collider.CompareTag ("Ai")) {
					//print (hit.collider.tag);
					Debug.DrawLine (sensorStartingPosition, hit.point);
					avoiding = true;
					avoidMultiplier -= 0.5f;
                    isCollidedOnReverse = false;
                }
			}

			//Front Left Sensor
			sensorStartingPosition -= transform.right * frontSideSensorPos * 2;
			if (Physics.Raycast (sensorStartingPosition, transform.forward, out hit, sensorLenght)) {
				if (!hit.collider.CompareTag ("Terrain") && !hit.collider.CompareTag ("Ai")) {
					//print (hit.collider.tag);
					Debug.DrawLine (sensorStartingPosition, hit.point);
					avoiding = true;
					avoidMultiplier += 1f;
                    isCollidedOnReverse = false;
                }
			} //Side Left Sensor
			else if (Physics.Raycast (sensorStartingPosition, Quaternion.AngleAxis(-frontSensorAngel, transform.up) * transform.forward, out hit, sensorLenght)) {
				if (!hit.collider.CompareTag ("Terrain") && !hit.collider.CompareTag ("Ai")) {
					//print (hit.collider.tag);
					Debug.DrawLine (sensorStartingPosition, hit.point);
					avoiding = true;
					avoidMultiplier += 0.5f;
                    isCollidedOnReverse = false;

                }
			}

            //Rear sensor
            if (Physics.Raycast (backSensorStartingPos, -transform.forward, out hit, sensorLenght)) {
				if (!hit.collider.CompareTag ("Terrain") && !hit.collider.CompareTag ("Ai")) {
					//print (hit.collider.tag);
					Debug.DrawLine (backSensorStartingPos, hit.point);
					avoiding = true;
                    isCollidedOnReverse = true;

                }
			}
			if (avoiding) {
                //Steering(maxSteerAngel * avoidMultiplier);
                //print(avoidMultiplier);
				targetSteerAngel = maxSteerAngel * avoidMultiplier;  
            }

           // print(hit.distance + " " + CurrentSpeed);

            if (hit.collider != null && !hit.collider.CompareTag("Ai") && hit.distance <= 1 && !isCollided)
            {
                avoiding = false;
                //print("HIT !!!");
                Stucked();
            }
        }

        void speedWhileCornering()
        {
            //print((int)steerWay);
           // targetSteerAngel = 2.5f * (int) steerWay;
           
            for (int i = 0; i < 4; i++)
            {
                m_WheelColliders[i].motorTorque = m_WheelColliders[i].motorTorque / torqueSteeringDiscount;
            }
        }

		void ApplySteer() {

			if (avoiding) {
				return;
			}
			Vector3 relativeVector = transform.InverseTransformPoint (wayPoints [_myWaypointIndex].position);
			float newSteer = (relativeVector.x / relativeVector.magnitude) * maxSteerAngel;
            //m_WheelColliders[0].steerAngle = newSteer;
            //m_WheelColliders[1].steerAngle = newSteer;
            targetSteerAngel = newSteer;
			//Steering(newSteer);
		}

		void Steering(float steer) {
			m_WheelColliders[0].steerAngle = steer;
			m_WheelColliders[1].steerAngle = steer;
		}

		void LerpToSteerAngel() {
			m_WheelColliders [0].steerAngle = Mathf.Lerp (m_WheelColliders [0].steerAngle, targetSteerAngel, Time.deltaTime * turnSpeed);
			m_WheelColliders [1].steerAngle = Mathf.Lerp (m_WheelColliders [1].steerAngle, targetSteerAngel, Time.deltaTime * turnSpeed);
		}

		void Movement() {

			for (int i = 0; i < 4; i++)
			{
				Quaternion quat;
				Vector3 position;
				m_WheelColliders[i].GetWorldPose(out position, out quat);
				m_WheelMeshes[i].transform.position = position;
				m_WheelMeshes[i].transform.rotation = quat;
			}

			float accel = Mathf.Clamp(AccelInput, 0, 1);
			float footbrake = -1*Mathf.Clamp(BrakeInput, -1, 0);

			/*currentSpeed = 2 * Mathf.PI * m_WheelColliders[0].radius * m_WheelColliders[0].rpm * 60 / 1000;

			float driveSpeed = 0;

			if (currentSpeed < maxSpeed && !isBraking) {
				driveSpeed = moveSpeed;
			} else {
				driveSpeed = 0;
			}*/

			//OLD CODE
			//m_WheelColliders[0].motorTorque = moveSpeed;
			//m_WheelColliders[1].motorTorque = moveSpeed;

            if(isCornering)
            {
                speedWhileCornering();
            } else {
                ApplyDrive(accel, footbrake);
            }
			
			CarSpeed ();

			CheckForWheelSpin();
			GearChanging();
			AddDownForce ();
			CalculateRevs ();


		}

		private void ApplyDrive(float accel, float footbrake)
		{
			//print (accel);
			float thrustTorque;

			AiCarBraking (footbrake);

			switch (m_CarDriveType)
			{
			case CarDriveType.FourWheelDrive:
				thrustTorque = accel * (m_CurrentTorque / 4f);
				for (int i = 0; i < 4; i++)
				{
					m_WheelColliders[i].motorTorque = thrustTorque;
				}
				break;

			case CarDriveType.FrontWheelDrive:
				thrustTorque = accel * (m_CurrentTorque / 2f);
			    m_WheelColliders [0].motorTorque = m_WheelColliders [1].motorTorque = thrustTorque; // * m_SlipLimit;
				break;

			case CarDriveType.RearWheelDrive:
				thrustTorque = accel * (m_CurrentTorque / 2f);
				m_WheelColliders[2].motorTorque = m_WheelColliders[3].motorTorque = thrustTorque;
				break;

			}

			/*for (int i = 0; i < 4; i++)
			{
				if (CurrentSpeed > 5 && Vector3.Angle(transform.forward, m_Rigidbody.velocity) < 50f)
				{
					m_WheelColliders[i].brakeTorque = m_BrakeTorque*footbrake;
				}
				else if (footbrake > 0)
				{
					m_WheelColliders[i].brakeTorque = 0f;
					m_WheelColliders[i].motorTorque = -m_ReverseTorque*footbrake;
				}
			}*/
		}

		private void AddDownForce()
		{
			m_WheelColliders[0].attachedRigidbody.AddForce(-transform.up*m_Downforce*
				m_WheelColliders[0].attachedRigidbody.velocity.magnitude);
		}

		private void CheckForWheelSpin()
		{
			// loop through all wheels
			for (int i = 0; i < 4; i++)
			{
				WheelHit wheelHit;
				m_WheelColliders[i].GetGroundHit(out wheelHit);

				// is the tire slipping above the given threshhold
				if (Mathf.Abs(wheelHit.forwardSlip) >= m_SlipLimit || Mathf.Abs(wheelHit.sidewaysSlip) >= m_SlipLimit)
				{
					m_WheelEffects[i].EmitTyreSmoke();

					// avoiding all four tires screeching at the same time
					// if they do it can lead to some strange audio artefacts
					if (!AnySkidSoundPlaying())
					{
						m_WheelEffects[i].PlayAudio();
					}
					continue;
				}

				// if it wasnt slipping stop all the audio
				if (m_WheelEffects[i].PlayingAudio)
				{
					m_WheelEffects[i].StopAudio();
				}
				// end the trail generation
				m_WheelEffects[i].EndSkidTrail();
			}
		}

		private void AiCarBraking(float brakeTorque) {

			if (isBraking) {
				//m_WheelColliders [2].brakeTorque = maxBrakeTorque;
				//m_WheelColliders [3].brakeTorque = maxBrakeTorque;
				for (int i = 0; i < 4; i++) {
					if (CurrentSpeed > 5 && Vector3.Angle (transform.forward, m_Rigidbody.velocity) < 50f) {
						m_WheelColliders [i].brakeTorque = maxBrakeTorque;//m_BrakeTorque * brakeTorque;
					} 
				}
			} else {
				for (int i = 0; i < 4; i++) {
					m_WheelColliders [i].brakeTorque = 0;
				}
			}
			/*else {
				m_WheelColliders [2].brakeTorque = 0;
				m_WheelColliders [3].brakeTorque = 0;
			}*/

			fireBrakeLights ();
		}

		private void GearChanging()
		{
			float f = Mathf.Abs(CurrentSpeed/maxSpeed);
			float upgearlimit = (1/(float) NoOfGears)*(m_GearNum + 1);
			float downgearlimit = (1/(float) NoOfGears)*m_GearNum;

			if (m_GearNum > 0 && f < downgearlimit)
			{
				m_GearNum--;
			}

			if (f > upgearlimit && (m_GearNum < (NoOfGears - 1)))
			{
				m_GearNum++;
			}
		}

		// simple function to add a curved bias towards 1 for a value in the 0-1 range
		private static float CurveFactor(float factor)
		{
			return 1 - (1 - factor)*(1 - factor);
		}


		// unclamped version of Lerp, to allow value to exceed the from-to range
		private static float ULerp(float from, float to, float value)
		{
			return (1.0f - value)*from + value*to;
		}


		private void CalculateGearFactor()
		{
			float f = (1/(float) NoOfGears);
			// gear factor is a normalised representation of the current speed within the current gear's range of speeds.
			// We smooth towards the 'target' gear factor, so that revs don't instantly snap up or down when changing gear.
			var targetGearFactor = Mathf.InverseLerp(f*m_GearNum, f*(m_GearNum + 1), Mathf.Abs(CurrentSpeed/maxSpeed));
			m_GearFactor = Mathf.Lerp(m_GearFactor, targetGearFactor, Time.deltaTime*5f);
		}


		private void CalculateRevs()
		{
			// calculate engine revs (for display / sound)
			// (this is done in retrospect - revs are not used in force/power calculations)
			CalculateGearFactor();
			var gearNumFactor = m_GearNum/(float) NoOfGears;
			var revsRangeMin = ULerp(0f, m_RevRangeBoundary, CurveFactor(gearNumFactor));
			var revsRangeMax = ULerp(m_RevRangeBoundary, 1f, gearNumFactor);
			Revs = ULerp(revsRangeMin, revsRangeMax, m_GearFactor);
		}

		private List<Transform> getPathTransform() {
			Transform[] pathTransform = path.GetComponentsInChildren<Transform> ();
			List<Transform> wayPoints = new List<Transform> ();
			for (int i = 0; i < pathTransform.Length; i++) {
				if (pathTransform [i] != path.transform) {
					wayPoints.Add (pathTransform [i]);
				}
			}
			return wayPoints;
		}

		void CheckWaypointDistance() {
			if(Vector3.Distance(transform.position, wayPoints[_myWaypointIndex].position) < 3.6f) {
				if (_myWaypointIndex == wayPoints.Count - 1) {
					_myWaypointIndex = 0;
				} else {
					_myWaypointIndex++;
				}
                if(checkpointSFX != null)
                {
                    AudioSource.PlayClipAtPoint(checkpointSFX, transform.position);
                }
                //AudioClip.
			}
		}
			
		void onOffLights ()
		{
			foreach (Light light in lights)
			{
				if (Input.GetKeyDown(keyboard))
				{
					light .enabled = !light .enabled;
				}
			}

		}

		void fireBrakeLights() {

			foreach (Light light in brakeLights) 
			{
				/*if (Input.GetKeyDown (keyboard)) 
					light.enabled = !light.enabled;
				{
					if (Input.GetKeyDown (keyboard2)) 
						light.enabled = !light.enabled;
					{
						if (Input.GetKeyUp (keyboard))
							light.enabled = !light.enabled;
						{
							if (Input.GetKeyUp (keyboard2))
								light.enabled = !light.enabled;

						}
					}
				}*/

				if (isBraking) {
					light.enabled = true;
				} else {
					light.enabled = false;
				}
			}
		}

		private bool AnySkidSoundPlaying()
		{
			for (int i = 0; i < 4; i++)
			{
				if (m_WheelEffects[i].PlayingAudio)
				{
					return true;
				}
			}
			return false;
		}

		private void CarSpeed()
		{
			float speed = m_Rigidbody.velocity.magnitude;
			switch (m_SpeedType)
			{
			case SpeedType.MPH:

				speed *= 2.23693629f;
				if (speed > maxSpeed)
					m_Rigidbody.velocity = (maxSpeed/2.23693629f) * m_Rigidbody.velocity.normalized;
				break;

			case SpeedType.KPH:
				speed *= 3.6f;
				if (speed > maxSpeed)
					m_Rigidbody.velocity = (maxSpeed/3.6f) * m_Rigidbody.velocity.normalized;
				break;
			}
		}

        void Stucked()
        {
            if (CurrentSpeed < 8f)
            {
                //print(CurrentSpeed);
                //print(collision.gameObject.name + " - " + collision.gameObject.tag + " Reverse !");
               
                isCollided = true;
                Unstuck();
               
                
                // CalculateRevs();
                //}
            }
        }

        void Unstuck()
        {
            // print("Reversing");

            for(int i=0; i < 4; i++)
            {
                m_WheelColliders[i].motorTorque = 0;
                m_WheelColliders[i].brakeTorque = maxBrakeTorque;
            }

            float appliedTorque = 4 * m_ReverseTorque;
            for (int i = 0; i < 4; i++)
            {
                m_WheelColliders[i].brakeTorque = 0;
                if(isCollidedOnReverse)
                {
                    m_WheelColliders[i].motorTorque = appliedTorque;
                } else
                {
                    m_WheelColliders[i].motorTorque = -appliedTorque;
                }
               
            }
               // m_WheelColliders[2].motorTorque = m_WheelColliders[3].motorTorque = -reverersTorque;
        }

        void StopReverse()
        {
            for (int i = 0; i < 4; i++)
            {
                m_WheelColliders[i].motorTorque = 0;
                m_WheelColliders[i].brakeTorque = maxBrakeTorque;
            }
        }

        private void OnCollisionEnter(Collision collision)
        {
            //   if(collision.gameObject.tag == "Obstacle")
            //{

            /*        if (collision.gameObject.tag.Equals("Water"))
                    {
                        _myWaypointIndex = 0;
                        transform.position = respawner.position;
                    }
                    else 

                    if(!collision.gameObject.tag.Equals("Obstacle") && !collision.gameObject.tag.Equals("Terrain") && !collision.gameObject.tag.Equals("Ai"))
                    {
                        print("Speed: " + CurrentSpeed);
                        print(collision.gameObject.name + " - " + collision.gameObject.tag + " Reverse !");
                        transform.position = wayPoints[_myWaypointIndex].position;
                            //SceneManager.LoadScene("race_track_lake");
                    } else
            {*/
            if (CurrentSpeed < 8f)
                {
                    isCollided = true;
                    Unstuck();
                }
           // }
        }

        internal enum SpeedType
		{
			MPH,
			KPH
		}

        internal enum CarDriveType
        {
            FourWheelDrive, FrontWheelDrive, RearWheelDrive
        }

    }
}
