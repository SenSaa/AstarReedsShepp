using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PreviousPositions : MonoBehaviour
{

    [SerializeField] private Transform AxlePrefab;
    private Transform PrevStatesObjsParent;
    private Vector3 PrevStatePos;

    void Start()
    {
        PrevStatesObjsParent = CreatePatentObj();
        StartCoroutine(StateTrace());
    }

    private Transform CreatePatentObj()
    {
        GameObject parentObj = new GameObject("PreviousStates");
        return parentObj.transform;
    }

    IEnumerator StateTrace()
    {
        while (true)
        {
            if(Vector3.Distance(PrevStatePos, transform.position) >= Params.CarLength * 0.8f)
            {
                Instantiate(AxlePrefab, transform.position, transform.rotation, PrevStatesObjsParent);
                PrevStatePos = transform.position;
            }
            yield return new WaitForSeconds(0.05f);
        }
    }

}
