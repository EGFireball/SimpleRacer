using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UnityStandardAssets.Vehicles.MyCar {

	public class Path : MonoBehaviour {

		public Color lineColor;
		private List<Transform> wayPoints = new List<Transform> ();
		[SerializeField] private float sphereRadius = 2.8f;

		// Use this for initialization
		void Start () {
			
		}
		
		// Update is called once per frame
		void Update () {
			
		}

		void OnDrawGizmos() {
			Gizmos.color = lineColor;

			/*Transform[] pathTransform = GetComponentsInChildren<Transform> ();

			wayPoints = new List<Transform> ();
			for (int i = 0; i < pathTransform.Length; i++) {
				if (pathTransform [i] != transform) {
					wayPoints.Add (pathTransform [i]);
				}
			}*/

			wayPoints = getPathTransform ();

			for (int i = 0; i < wayPoints.Count; i++) {
				Vector3 currentPoint = wayPoints [i].position;
				Vector3 previousPoint = Vector3.zero;
				if (i > 0) { 
					previousPoint = wayPoints [i - 1].position;
				} else if( i == 0 && wayPoints.Count > 1 ){
					previousPoint = wayPoints [wayPoints.Count - 1].position;
				}

				Gizmos.DrawLine (previousPoint, currentPoint);
				Gizmos.DrawWireSphere (currentPoint, sphereRadius);
			}
		}

		public List<Transform> getPathTransform() {
			Transform[] pathTransform = GetComponentsInChildren<Transform> ();

			List<Transform> wayPoints = new List<Transform> ();
			for (int i = 0; i < pathTransform.Length; i++) {
				if (pathTransform [i] != transform) {
					wayPoints.Add (pathTransform [i]);
				}
			}
			return wayPoints;
		}
	}
}
