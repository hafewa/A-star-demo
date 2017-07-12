using UnityEngine;
using UnityEngine.UI;

public class ToggleController : MonoBehaviour {

    private void Start()
    {
        if(GridManager.Instance.CanDiagonalMove)
        {
            GetComponent<Toggle>().isOn = GridManager.Instance.CanDiagonalMove;
        }
    }

    public void SetDiagonalMove(bool isCan)
    {
        GridManager.Instance.CanDiagonalMove = isCan;
    }

}
