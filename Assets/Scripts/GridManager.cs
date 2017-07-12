using UnityEngine;
using PathSearch.Astar;

public class GridManager : MonoBehaviour {

    public int Width = 10;
    public int Height = 10;
    public bool CanDiagonalMove = false;
    public GameObject GridPrefab;

    public static GameObject[,] GridDate;
    public static Point[,] PointGrid;

    public static Point StartPoint;

    private static GridManager instance;
    public static GridManager Instance
    {
        get { return instance; }
    }

    private void Awake()
    {
        instance = this;
    }

    private void Start()
    {
        GridDate = new GameObject[Width, Height];
        PointGrid = new Point[Width, Height];
        CreateGrid();
        RandomStartPoint();
    }

    public void CreateGrid()
    {
        for (int x = 0; x < Width; x++)
        {
            for (int y = 0; y < Height; y++)
            {
                GameObject grid = Instantiate(GridPrefab, new Vector3(transform.position.x + x, transform.position.y + y, 0), Quaternion.identity);
                Grid g = grid.GetComponent<Grid>();
                g.X = x;
                g.Y = y;
                GridDate[x, y] = grid;
                PointGrid[x, y] = new Point(x, y);
                bool canNotPass = Random.value < 0.1f;
                if(canNotPass)
                {
                    GridDate[x, y].GetComponent<SpriteRenderer>().color = Color.red;
                    PointGrid[x, y].CanPass = false;
                }
            }
        }
    }

    private void RandomStartPoint()
    {
        int x = Random.Range(0, Width - 1);
        int y = Random.Range(0, Height - 1);
        StartPoint = PointGrid[x, y];
        PointGrid[x, y].CanPass = true;
        GridDate[x, y].GetComponent<SpriteRenderer>().color = Color.blue;
    }

}
