using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UnityStandardAssets.Vehicles.MyCar {

	public class SmoothCamera : MonoBehaviour {

		public Transform target;

		public float distance = 10f;
		public float height = 5f;

		float heightDamping = 2.0f;
		float rotationDamping = 3.0f;


		// Use this for initialization
		void Start () {
			
		}
		
		// Update is called once per frame
		void Update () {
			
		}

		void LateUpdate () {
			// Early out if we don’t have a target  if (!target)  return;
			// Calculate the current rotation angles 
			float wantedRotationAngle = target.eulerAngles.y; 
			float wantedHeight = target.position.y + height;
			float currentRotationAngle = transform.eulerAngles.y; 
			float currentHeight = transform.position.y;
			// Damp the rotation around the y-axis
			currentRotationAngle = Mathf.LerpAngle (currentRotationAngle, wantedRotationAngle, rotationDamping * Time.deltaTime);
			// Damp the height
			currentHeight = Mathf.Lerp (currentHeight, wantedHeight, heightDamping * Time.deltaTime);
			// Convert the angle into a rotation. The // quaternion interface uses radians not degrees  // so we need to convert from degrees to radians
			Quaternion currentRotation = Quaternion.EulerAngles (0, currentRotationAngle * Mathf.Deg2Rad, 0);
			// Set the position of camera on the x-z plane to: // distance meters behind the target
			transform.position = target.position; 
			transform.position -= currentRotation * Vector3. forward * distance;


			// Set the height of the camera 
			//transform.position.y = currentHeight;
			transform.position = new Vector3(transform.position.x, currentHeight, transform.position.z);
			// Always look at the target 
			transform.LookAt (target); 
		}
	}
}
