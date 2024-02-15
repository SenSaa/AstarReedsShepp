using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using astar;

public class BroadcastData
{

    public void BroadcastSearchUpdate(Node node, Queue<Node> nodeQ, int i)
    {
        SearchUpdate searchUpdate = new SearchUpdate
        {
            CurrentNode = node,
            NodeQueue = nodeQ,
            Iteration = i
        };
        MessageBroker.SearchUpdateEventArgs searchUpdateEventArgs = new MessageBroker.SearchUpdateEventArgs
        {
            Update = searchUpdate
        };
        MessageBroker messageBroker = new MessageBroker();
        messageBroker.OnSearchUpdateEvent(searchUpdateEventArgs);
    }

    public void BroadcastSearchResults(bool success, List<Node> path)
    {
        SearchResult searchResult = new SearchResult
        {
            success = success,
            Path = path
        };
        MessageBroker.SearchResultsEventArgs searchResultsEventArgs = new MessageBroker.SearchResultsEventArgs
        {
            Results = searchResult
        };
        MessageBroker messageBroker = new MessageBroker();
        messageBroker.OnSearchResultsEvent(searchResultsEventArgs);
    }

    public void BroadcastCurrentRSpath(Path rsPath)
    {
        MessageBroker.CurrentRSpathEventArgs currentRSpathEventArgs = new MessageBroker.CurrentRSpathEventArgs
        {
            CurrentRSpath = rsPath
        };
        MessageBroker messageBroker = new MessageBroker();
        messageBroker.OnCurrentRSpathUpdateEvent(currentRSpathEventArgs);
    }

    public void BroadcastSmoothPath(Path finalPath)
    {
        MessageBroker.SmoothedPathEventArgs smoothedPathEventArgs = new MessageBroker.SmoothedPathEventArgs
        {
            Path = finalPath
        };
        MessageBroker messageBroker = new MessageBroker();
        messageBroker.OnPathEvent(smoothedPathEventArgs);
    }

    public void BroadcastBehaviourPlanUpdate(VehicleControlCommandFlags vehicleControlCommandFlags)
    {
        MessageBroker.BehaviourPlanEventArgs behaviourPlanEventArgs = new MessageBroker.BehaviourPlanEventArgs
        {
            VehicleControlCommandFlags = vehicleControlCommandFlags
        };
        MessageBroker messageBroker = new MessageBroker();
        messageBroker.BehaviourPlanUpdate(behaviourPlanEventArgs);
    }


    public void BroadcastConfiguration(Vector3 StartPos, float StartRotation, Vector3 GoalPos, float GoalRotation)
    {
        MessageBroker.ConfigurationEventArgs configurationEventArgs = new MessageBroker.ConfigurationEventArgs()
        {
            StartPos = StartPos,
            StartYaw = StartRotation,
            GoalPos = GoalPos,
            GoalYaw = GoalRotation
        };
        MessageBroker messageBroker = new MessageBroker();
        messageBroker.ConfigurationEvent(configurationEventArgs);
    }

}
