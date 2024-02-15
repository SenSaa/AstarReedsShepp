using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UtilityFunctions;
using UnityEngine.UI;

// Path Tracking using Pure Pursuit steering control and PID speed controller.

namespace PurePursuit
{

    public class Controller : MonoBehaviour
    {

        [SerializeField] private List<WheelCollider> WheelColliders; // Order -> FL, FR, RL, RR.
        [SerializeField] private Transform Agent;
        [SerializeField] private Transform AgentCenterPos;
        [SerializeField] private Material TargetMat;
        [SerializeField] private Text SpeedText;
        [SerializeField] private Text SteerText;
        private Path ParameteisedPath = new Path(); // Target course
        private Transform Target;
        private Transform CenterPos;
        [SerializeField] private Vector3 GoalPos;
        [SerializeField] private float GoalYaw;
        private Rigidbody rigidbody;
        [SerializeField] private float MotorTorque = 2000;
        [SerializeField] private float BrakeTorque = 2000;
        [SerializeField] private float TargetSpeed_kmh;
        [SerializeField] private float CurrentSpeed_kmh;
        [SerializeField] float SteeringAngleCommand;
        [SerializeField] float CurrentSteeringAngle;
        private bool ControlInitComplete;
        private BehaviourPlanner behaviourPlanner;
        [SerializeField] private FSM.State BehaviourState;
        private SpeedControl speedControl;
        private PurePursuitControl purePursuitControl;
        private VehicleControls vehicleControls;
        private State state;
        private Visualisations visualisations;
        [SerializeField] private int TargetIndex;
        [SerializeField] private int PathDir;
        VehicleControlCommandFlags vehicleControlCommandFlags;


        private void Awake()
        {
            MessageBroker.Path += HandlePathEvent;
            MessageBroker.Configuration += HandleConfigurationEvent;
            MessageBroker.BehaviourPlan += HandleBehaviourPlanUpdate;
        }

        private void HandlePathEvent(object o, MessageBroker.SmoothedPathEventArgs e)
        {
            ParameteisedPath = e.Path;
        }

        private void HandleConfigurationEvent(object o, MessageBroker.ConfigurationEventArgs e)
        {
            GoalPos = e.GoalPos;
            GoalYaw = e.GoalYaw;
        }

        private void HandleBehaviourPlanUpdate(object o, MessageBroker.BehaviourPlanEventArgs e)
        {
            vehicleControlCommandFlags = e.VehicleControlCommandFlags;
        }


        void Start()
        {
            if (Agent == null)
            {
                Agent = transform;
            }

            rigidbody = GetComponent<Rigidbody>();

            behaviourPlanner = new BehaviourPlanner();
            speedControl = new SpeedControl();
            purePursuitControl = new PurePursuitControl();
            vehicleControls = new VehicleControls(WheelColliders, MotorTorque, BrakeTorque);
            vehicleControlCommandFlags = new VehicleControlCommandFlags();
            visualisations = new Visualisations();

            Target = visualisations.renderObject("Target", TargetMat);
            CenterPos = visualisations.renderObject("CenterPos");
        }

        void FixedUpdate()
        {
            if (ParameteisedPath.x.Count < 1) { return; }

            if (!ControlInitComplete) { ControlInitComplete = !ControlInitComplete; InitControl(); }

            ControlUpdate();
        }


        private void InitControl()
        {
            behaviourPlanner.InitState(ParameteisedPath);
            purePursuitControl.PathTrackingInit(Agent, ParameteisedPath);
        }


        private void ControlUpdate()
        {
            // Retrieve directions from RS path!
            PathDir = ParameteisedPath.directions[TargetIndex];

            // Steering Control
            // Pure Pursuit
            SteeringControl();

            // Behaviour State Update
            float targetSpeed_ms = BehaviourUpdate();

            // Speed Control
            // PID
            double speedPidOutput = SpeedControl(targetSpeed_ms);

            // Apply Control Commands
            VehicleControl(speedPidOutput);

            // Visualisations
            VisualisePathTracking();

            // UI
            UIupdate();
        }

        private void SteeringControl()
        {
            var state_index = purePursuitControl.PathTrackingUpdate(PathDir);
            state = state_index.state;
            TargetIndex = state_index.targetInd;
        }

        private float BehaviourUpdate()
        {
            var targetSpeed_ms = behaviourPlanner.StateUpdate(state, TargetIndex, PathDir);
            TargetSpeed_kmh = Mathf.Round((float)targetSpeed_ms * 3.6f);
            BehaviourState = behaviourPlanner.GetBehaviourState();
            return (float)targetSpeed_ms;
        }

        private double SpeedControl(float targetSpeed_ms)
        {
            var speedControlOutput = speedControl.Update(targetSpeed_ms, Agent, rigidbody);
            double speedPidOutput = speedControlOutput.pidOp;
            CurrentSpeed_kmh = Mathf.Round(speedControlOutput.currSpd);
            return speedPidOutput;
        }

        private void VehicleControl(double speedPidOutput)
        {
            vehicleControls.ApplySpeed((float)speedPidOutput);
            vehicleControls.ApplySteer(state.delta, PathDir);

            if(vehicleControlCommandFlags.ApplyBrake) { vehicleControls.ApplyBrake(); }
            if (vehicleControlCommandFlags.ReleaseBrake) { vehicleControls.ReleaseBrake(); }
            if (vehicleControlCommandFlags.ReleaseThrottle) { vehicleControls.ReleaseThrottle(); }
            if (vehicleControlCommandFlags.ReleaseSteering) { vehicleControls.ReleaseSteering(); }

            SteeringAngleCommand = Mathf.Round((float)state.delta);
            CurrentSteeringAngle = Mathf.Round(Utils.GetCurrentSteeringAngle(WheelColliders[0]));
        }

        private void VisualisePathTracking()
        {
            visualisations.updateObject(Target, ParameteisedPath.x[TargetIndex], ParameteisedPath.y[TargetIndex]);
            visualisations.updateObject(CenterPos, state.x, state.y);
        }

        private void UIupdate()
        {
            SpeedText.text = "Speed: " + CurrentSpeed_kmh;
            SteerText.text = "Steer: " + CurrentSteeringAngle;
        }

    }

}
