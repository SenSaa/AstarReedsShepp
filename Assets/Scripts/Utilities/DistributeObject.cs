using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

// Scrpt for generating gameobjects based on.
// reference position, number of objects to create,
// Step increments, and specified direction (global right/left, up/down).
// * Makes generating obstacles in environment fast!

public class DistributeObject : MonoBehaviour
{

    [SerializeField] private Transform ReferenceTransform; // Object Transform used as a starting point.
    [SerializeField] private int HowManyObjectsToCreate;
    [SerializeField] private int PosIncrement; // How much to shift position of object by.
    [SerializeField] private bool X; // Trigger in inspector if you want to shift by x!
    [SerializeField] private bool Z; // Trigger in inspector if you want to shift by z!

    void Start()
    {
        if(!X && !Z) { return; }
        Vector3 prevPosition = ReferenceTransform.position;
        for (int i = 0; i < HowManyObjectsToCreate; i++)
        {
            var clone = Instantiate(ReferenceTransform);
            clone.parent = ReferenceTransform.parent;
            clone.name = ReferenceTransform.name;
            clone.position = ChangePosition(prevPosition);
            prevPosition.Set(clone.position.x, clone.position.y, clone.position.z);
        }
    }

    private Vector3 ChangePosition(Vector3 position)
    {
        if (X && Z)
        {
            position.Set(position.x + PosIncrement, position.y, position.z + PosIncrement);
        }
        if (X)
        {
            position.Set(position.x + PosIncrement, position.y, position.z);
        }
        if (Z)
        {
            position.Set(position.x, position.y, position.z + PosIncrement);
        }
        return position;
    }
}
