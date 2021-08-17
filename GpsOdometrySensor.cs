/**
 * Copyright (c) 2018-2021 LG Electronics, Inc.
 *
 * This software contains code licensed as described in LICENSE.
 *
 */

using Simulator.Bridge;
using Simulator.Bridge.Data;
using Simulator.Map;
using Simulator.Utilities;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using Simulator.Sensors.UI;
using Simulator.Analysis;

namespace Simulator.Sensors
{
    [SensorType("GPS Odometry", new[] { typeof(GpsOdometryData) })]
    public class GpsOdometrySensor : SensorBase
    {
        [SensorParameter]
        [Range(1.0f, 100f)]
        public float Frequency = 12.5f;

        [SensorParameter]
        public string ChildFrame;

        [SensorParameter]
        public bool IgnoreMapOrigin = false;

        private Queue<Tuple<double, Action>> MessageQueue = new Queue<Tuple<double, Action>>();

        private bool Destroyed = false;
        private bool IsFirstFixedUpdate = true;
        private double LastTimestamp;

        private float NextSend;
        private uint SendSequence;

        private BridgeInstance Bridge;
        private Publisher<GpsOdometryData> Publish;

        private IVehicleDynamics Dynamics;
        private MapOrigin MapOrigin;
        private Vector3 startPosition;

        public override SensorDistributionType DistributionType => SensorDistributionType.MainOrClient;
        public override float PerformanceLoad { get; } = 0.05f;

        private void Awake()
        {
            Dynamics = GetComponentInParent<IVehicleDynamics>();
            MapOrigin = MapOrigin.Find();
            startPosition = transform.position;
        }

        protected override void OnDestroy()
        {
            base.OnDestroy();
            Destroyed = true;
        }

        protected override void Initialize()
        {
            Task.Run(Publisher);
        }

        protected override void Deinitialize()
        {
            
        }

        public override void OnBridgeSetup(BridgeInstance bridge)
        {
            Bridge = bridge;
            Publish = Bridge.AddPublisher<GpsOdometryData>(Topic);
        }

        void Publisher()
        {
            var nextPublish = Stopwatch.GetTimestamp();

            while (!Destroyed)
            {
                long now = Stopwatch.GetTimestamp();
                if (now < nextPublish)
                {
                    Thread.Sleep(0);
                    continue;
                }

                Tuple<double, Action> msg = null;
                lock (MessageQueue)
                {
                    if (MessageQueue.Count > 0)
                    {
                        msg = MessageQueue.Dequeue();
                    }
                }

                if (msg != null)
                {
                    try
                    {
                        msg.Item2();
                    }
                    catch (Exception ex)
                    {
                        UnityEngine.Debug.LogException(ex, this);
                    }
                    nextPublish = now + (long)(Stopwatch.Frequency / Frequency);
                    LastTimestamp = msg.Item1;
                }
            }
        }


        void FixedUpdate()
        {
            if (MapOrigin == null || Bridge == null || Bridge.Status != Status.Connected)
            {
                return;
            }

            if (IsFirstFixedUpdate)
            {
                lock (MessageQueue)
                {
                    MessageQueue.Clear();
                }
                IsFirstFixedUpdate = false;
            }

            var time = SimulatorManager.Instance.CurrentTime;
            if (time < LastTimestamp)
            {
                return;
            }

            var location = MapOrigin.PositionToGpsLocation(transform.position, IgnoreMapOrigin);

            var orientation = transform.rotation;
            orientation.Set(-orientation.z, orientation.x, -orientation.y, orientation.w); // converting to right handed xyz

            var angularVelocity = Dynamics.AngularVelocity;
            angularVelocity.Set(-angularVelocity.z, angularVelocity.x, -angularVelocity.y); // converting to right handed xyz

            var data = new GpsOdometryData()
            {
                Name = Name,
                Frame = Frame,
                Time = SimulatorManager.Instance.CurrentTime,
                Sequence = SendSequence++,

                ChildFrame = ChildFrame,
                IgnoreMapOrigin = IgnoreMapOrigin,
                Latitude = location.Latitude,
                Longitude = location.Longitude,
                Altitude = location.Altitude,
                Northing = location.Northing,
                Easting = location.Easting,
                Orientation = orientation,
                ForwardSpeed = Vector3.Dot(Dynamics.Velocity, transform.forward),
                Velocity = Dynamics.Velocity,
                AngularVelocity = angularVelocity,
                WheelAngle = Dynamics.WheelAngle,
            };
            
            lock (MessageQueue)
            {
                MessageQueue.Enqueue(Tuple.Create(time, (Action)(() =>
                {
                    if (Bridge != null && Bridge.Status == Status.Connected)
                    {
                        Publish(data);
                    }
                })));
            }
        }

        void Update()
        {
            IsFirstFixedUpdate = true;
        }

        public override void OnVisualize(Visualizer visualizer)
        {
            UnityEngine.Debug.Assert(visualizer != null);

            var location = MapOrigin.PositionToGpsLocation(transform.position, IgnoreMapOrigin);

            var orientation = transform.rotation;
            orientation.Set(-orientation.z, orientation.x, -orientation.y, orientation.w); // converting to right handed xyz

            var angularVelocity = Dynamics.AngularVelocity;
            angularVelocity.Set(-angularVelocity.z, angularVelocity.x, -angularVelocity.y); // converting to right handed xyz

            var graphData = new Dictionary<string, object>()
            {
                {"Child Frame", ChildFrame},
                {"Ignore MapOrigin", IgnoreMapOrigin},
                {"Latitude", location.Latitude},
                {"Longitude", location.Longitude},
                {"Altitude", location.Altitude},
                {"Northing", location.Northing},
                {"Easting", location.Easting},
                {"Orientation", orientation},
                {"Forward Speed", Vector3.Dot(Dynamics.Velocity, transform.forward)},
                {"Velocity", Dynamics.Velocity},
                {"Angular Velocity", angularVelocity}
            };
            visualizer.UpdateGraphValues(graphData);
        }

        public override void OnVisualizeToggle(bool state)
        {
            //
        }

        public override void SetAnalysisData()
        {
            var startLocation = MapOrigin.PositionToGpsLocation(startPosition, IgnoreMapOrigin);
            var location = MapOrigin.PositionToGpsLocation(transform.position, IgnoreMapOrigin);

            SensorAnalysisData = new List<AnalysisReportItem>
            {
                new AnalysisReportItem {
                    name = "Start Latitude",
                    type = MeasurementType.Latitude,
                    value = startLocation.Latitude
                },
                new AnalysisReportItem {
                    name = "Start Longitude",
                    type = MeasurementType.Longitude,
                    value = startLocation.Longitude
                },
                new AnalysisReportItem {
                    name = "Start Altitude",
                    type = MeasurementType.Altitude,
                    value = startLocation.Altitude
                },
                new AnalysisReportItem {
                    name = "Start Northing",
                    type = MeasurementType.Northing,
                    value = startLocation.Northing
                },
                new AnalysisReportItem {
                    name = "Start Easting",
                    type = MeasurementType.Easting,
                    value = startLocation.Easting
                },
                new AnalysisReportItem {
                    name = "Start Map Url",
                    type = MeasurementType.MapURL,
                    value =  $"https://www.google.com/maps/search/?api=1&query={startLocation.Latitude},{startLocation.Longitude}"
                },
                new AnalysisReportItem {
                    name = "Latitude",
                    type = MeasurementType.Latitude,
                    value = location.Latitude
                },
                new AnalysisReportItem {
                    name = "Longitude",
                    type = MeasurementType.Longitude,
                    value = location.Longitude
                },
                new AnalysisReportItem {
                    name = "Altitude",
                    type = MeasurementType.Altitude,
                    value = location.Altitude
                },
                new AnalysisReportItem {
                    name = "Northing",
                    type = MeasurementType.Northing,
                    value = location.Northing
                },
                new AnalysisReportItem {
                    name = "Easting",
                    type = MeasurementType.Easting,
                    value = location.Easting
                },
                new AnalysisReportItem {
                    name = "Map Url",
                    type = MeasurementType.MapURL,
                    value =  $"https://www.google.com/maps/search/?api=1&query={location.Latitude},{location.Longitude}"
                },
            };
        }
    }
}
