using System.Diagnostics;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MouseFollow : MonoBehaviour
{
    public Camera UserCamera;
    private GameObject Bot;
    private Vector3 mouseToWorldPosition;
    
    void Start()
    {
        UserCamera = Camera.main;
        Bot = GameObject.Find("Bot");
    }

    void Update()
    {
        Vector3 mousePos = Input.mousePosition;
        mousePos.z = 10;
        mouseToWorldPosition = UserCamera.ScreenToWorldPoint(mousePos);
        // UnityEngine.Debug.Log(mouseToWorldPosition);
        Bot.transform.localPosition = mouseToWorldPosition;
    }
}
