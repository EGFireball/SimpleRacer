using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UnityStandardAssets.Vehicles.MyCar
{
    public class WayPoint : MonoBehaviour
    {

        [SerializeField] private bool isSlowPoint;

        public bool IsSlowPoint()
        {
            return this.isSlowPoint;
        }
    }
}