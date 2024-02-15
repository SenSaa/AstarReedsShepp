using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MessageBroker
{

    public static event EventHandler<SmoothedPathEventArgs> Path;
    public static event EventHandler<SearchResultsEventArgs> SearchResults;
    public static event EventHandler<SearchUpdateEventArgs> SearchUpdate;
    public static event EventHandler<CurrentRSpathEventArgs> CurrentRSpath;
    public static event EventHandler<BehaviourPlanEventArgs> BehaviourPlan;
    public static event EventHandler<ConfigurationEventArgs> Configuration;


    public void OnPathEvent(SmoothedPathEventArgs e)
    {
        Path?.Invoke(this, e);
    }

    public void OnSearchResultsEvent(SearchResultsEventArgs e)
    {
        SearchResults?.Invoke(this, e);
    }

    public void OnSearchUpdateEvent(SearchUpdateEventArgs e)
    {
        SearchUpdate?.Invoke(this, e);
    }

    public void OnCurrentRSpathUpdateEvent(CurrentRSpathEventArgs e)
    {
        CurrentRSpath?.Invoke(this, e);
    }

    public void BehaviourPlanUpdate(BehaviourPlanEventArgs e)
    {
        BehaviourPlan?.Invoke(this, e);
    }

    public void ConfigurationEvent(ConfigurationEventArgs e)
    {
        Configuration?.Invoke(this, e);
    }


    public class SmoothedPathEventArgs : EventArgs
    {
        public Path Path { get; set; }
        public List<Vector3> SmoothPath { get; set; }
    }

    public class SearchResultsEventArgs : EventArgs
    {
        public SearchResult Results { get; set; }
    }

    public class SearchUpdateEventArgs : EventArgs
    {
        public SearchUpdate Update { get; set; }
    }

    public class CurrentRSpathEventArgs : EventArgs
    {
        public Path CurrentRSpath { get; set; }
    }

    public class BehaviourPlanEventArgs : EventArgs
    {
        public VehicleControlCommandFlags VehicleControlCommandFlags { get; set; }
    }

    public class ConfigurationEventArgs : EventArgs
    {
        public Vector3 StartPos { get; set; }
        public Vector3 GoalPos { get; set; }
        public float StartYaw { get; set; }
        public float GoalYaw { get; set; }
    }

}
