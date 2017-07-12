using UnityEngine;
using PathSearch.Astar;

public class Grid : MonoBehaviour {

    public int X;
    public int Y;

    private void OnMouseUpAsButton()
    {
        Point endPoint = new Point(X, Y);
        Point startPoint = GridManager.StartPoint;
        Searcher s = new Searcher(startPoint, endPoint, GridManager.PointGrid, GridManager.Instance.CanDiagonalMove);
        Point target = s.FindPath();
        if (target != null)
        {
            while (target.Parent != null)
            {
                int x = target.X;
                int y = target.Y;
                GridManager.GridDate[x, y].GetComponent<SpriteRenderer>().color = Color.green;
                target = target.Parent;
            }
        }
        else
        {
            Debug.Log("没有找到路径");
        }
    }

}
