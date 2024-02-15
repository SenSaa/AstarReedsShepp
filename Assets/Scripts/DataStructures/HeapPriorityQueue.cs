using System;
using System.Collections.Generic;
using UnityEngine;

// Priority Queue implementation using a min Heap.
// Used the max heap implementation from link below (with some modifications).
// https://www.programiz.com/dsa/heap-data-structure

public class HeapPriorityQueue<T>
{

    private List<T> hT;
    private Comparison<T> comparison;

    public HeapPriorityQueue(Comparison<T> comparison)
    {
        hT = new List<T>();
        this.comparison = comparison;
    }

    private void heapify(int i)
    {
        int size = hT.Count;
        int smallest = i;
        int l = 2 * i + 1;
        int r = 2 * i + 2;
        if (l < size && comparison(hT[l], hT[smallest]) < 0)
            smallest = l;
        if (r < size && comparison(hT[r], hT[smallest]) < 0)
                smallest = r;

        if (smallest != i)
        {
            var temp = hT[smallest];
            hT[smallest] = hT[i];
            hT[i] = temp;

            heapify(smallest);
        }
    }

    public void Insert(T newNum)
    {
        int size = hT.Count;
        if (size == 0)
        {
            hT.Add(newNum);
        }
        else
        {
            hT.Add(newNum);

            // Perform a bubble-up operation to maintain the min heap property
            int currentIndex = size;
            int parentIndex = (currentIndex - 1) / 2;

            while (currentIndex > 0 && comparison(hT[currentIndex], hT[parentIndex]) < 0)
            {
                // Swap the current element with its parent
                var temp = hT[parentIndex];
                hT[parentIndex] = hT[currentIndex];
                hT[currentIndex] = temp;

                currentIndex = parentIndex;
                parentIndex = (currentIndex - 1) / 2;
            }
        }
    }

    public void Delete(T num)
    {
        int size = hT.Count;
        int i;
        for (i = 0; i < size; i++)
        {
            if (comparison(num, hT[i]) == 0)
                break;
        }

        var temp = hT[i];
        hT[i] = hT[size - 1];
        hT[size - 1] = temp;

        hT.RemoveAt(size - 1);
        for (int j = size / 2 - 1; j >= 0; j--)
        {
            heapify(j);
        }
    }

    public T Delete()
    {
        int size = hT.Count;
        if (size <= 0)
        {
            Debug.Log("Heap is empty. Cannot delete minimum element.");
            return default;
        }

        var temp = hT[0];
        hT[0] = hT[size - 1];
        hT[size - 1] = temp;

        hT.RemoveAt(size - 1);
        heapify(0);

        return temp;
    }

    public int GetSize()
    {
        return hT.Count;
    }

    public void print()
    {
        foreach (var i in hT)
        {
            Debug.Log(i + " ");
        }
    }

}
