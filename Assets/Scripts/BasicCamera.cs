using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UnityStandardAssets.Vehicles.MyCar {

	public class BasicCamera : MonoBehaviour {

		public Transform target;
		public float distance = 10f;
		float height = 10f;

		// Use this for initialization
		void Start () {
			
		}
		
		// Update is called once per frame
		void Update () {
			
		}

		// LateUpdate is like Update but always called 
		// after all Update functions on all scripts  
		// are called. 
		// This allows you to order script execution  
		// in the same frame. // Cameras should always be updated last.

		void LateUpdate() {
			// Early out if we don’t have a target  
			if (!target)  return;

			Vector3 forward = target.TransformDirection (Vector3.forward);

			Vector3 targetPosition = target.position;

			// Place the camera distance behind the target 
			transform.position = targetPosition - forward * distance;

			//move camera a bit up
			float currentHeight = height;
			transform.position = new Vector3(transform.position.x, currentHeight, transform.position.z);

			//always look at the target
			transform.LookAt (target);

		}

	}
}
