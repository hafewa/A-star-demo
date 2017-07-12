using System;
using System.Collections.Generic;

namespace PathSearch
{
    namespace Astar
    {
        public class Point : IComparable<Point>
        {
            // 从开始点, 经由N点(即本点)到目标点的估计代价
            public int F
            {
                get { return g + h; }
            }
            // 从初始点到本点时的实际代价
            private int g;
            public int G
            {
                get { return g; }
                set { g = value; }
            }
            // 从本点到目标点的最佳路径的估计代价
            private int h;
            public int H
            {
                get { return h; }
            }

            public Point Parent { get; set; }
            public bool CanPass = true;

            private int x;
            public int X
            {
                get { return x; }
            }

            private int y;
            public int Y
            {
                get { return y; }
            }

            public Point(int x, int y)
            {
                this.x = x;
                this.y = y;
            }

            public Point (int x, int y, bool canPass): this(x, y)
            {
                this.CanPass = canPass;
            }

            /// <summary>
            /// 计算本点和目标点之间的曼哈顿距离
            /// </summary>
            /// <param name="target"></param>
            /// <returns></returns>
            private int ManhattanDistance(Point target)
            {
                return Math.Abs(target.x - this.x) + Math.Abs(target.y - this.y);
            }

            /// <summary>
            /// 欧几里得距离, 即直线距离
            /// </summary>
            /// <param name="target"></param>
            /// <returns></returns>
            private int EuclideanDistance(Point target, bool useSqrt = false)
            {
                int distance = (int)(Math.Pow(target.x - this.x, 2.0) + Math.Pow(target.y - this.y,  2.0));
                if (useSqrt)
                {
                    distance = (int)Math.Sqrt(distance);
                }
                return distance;
            }

            /// <summary>
            /// 计算估值F
            /// </summary>
            /// <param name="startPoint"></param>
            /// <param name="endPoint"></param>
            /// <returns></returns>
            public int CalculateF(Point startPoint, Point endPoint)
            {
                g = ManhattanDistance(startPoint);
                h = EuclideanDistance(endPoint);
                return g + h;
            }

            /// <summary>
            /// 实现同类型对象的比较
            /// </summary>
            /// <param name="other"></param>
            /// <returns></returns>
            public int CompareTo(Point other)
            {
                if (this.F > other.F)
                {
                    return 1;
                }
                else if (this.F < other.F)
                {
                    return -1;
                }
                else
                {
                    return 0;
                }
            }

            /// <summary>
            /// 重写相等的判断, 用于List中的Contains方法, 注意, 并没有重载 == 符合的行为
            /// </summary>
            /// <param name="obj"></param>
            /// <returns></returns>
            public override bool Equals(object obj)
            {
                Point other = obj as Point;
                if (other == null)
                {
                    return base.Equals(obj);
                }
                if (x == other.x && y == other.y)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }

            /// <summary>
            /// 为了vs不报警告加上的, 无实际意义
            /// </summary>
            /// <returns></returns>
            public override int GetHashCode()
            {
                return base.GetHashCode();
            }
        } // end class

        public class Searcher
        {
            private Point startPoint;
            private Point endPoint;
            private Point[,] gridDate;
            private int gridWidth;
            private int gridHeight;
            private List<Point> openList = new List<Point>();
            private List<Point> closeList = new List<Point>();
            private bool canDiagonalMove = true; // 是否允许斜线移动

            public Searcher(Point startPoint, Point endPoint, Point[,] gridDate, bool canDiagonalMove = true)
            {
                this.startPoint = startPoint;
                this.endPoint = endPoint;
                this.canDiagonalMove = canDiagonalMove;
                this.gridDate = gridDate;
                gridWidth = gridDate.GetLength(1);
                gridHeight = gridDate.GetUpperBound(0) + 1;
            }

            public Searcher(Point startPoint, Point endPoint, Point[,] gridDate, int gridWidth, int gridHeight, bool canDiagonalMove = true)
            {
                this.startPoint = startPoint;
                this.endPoint = endPoint;
                this.canDiagonalMove = canDiagonalMove;
                this.gridDate = gridDate;
                this.gridWidth = gridWidth;
                this.gridHeight = gridHeight;
            }

            public Point FindPath()
            {
                // 若目标点不能到达, 返回null表示, 寻路结束
                if (!endPoint.CanPass)
                {
                    return null;
                }
                startPoint.CalculateF(startPoint, endPoint);
                openList.Add(startPoint);
                while(openList.Count != 0)
                {
                    // 获取openlist中的第一个点, 即F估值最小的点
                    Point choicePoint = openList[0];
                    // 如果是目标点, 说明找到了, 立即返回
                    if (choicePoint.Equals(endPoint))
                    {
                        return choicePoint;
                    }
                    openList.RemoveAt(0);
                    // 获取该点的周围可到达的点, 计算其估值F, 选取其中最小的, 加入的closelist中
                    List<Point> aroundPoints;
                    FindAroundPoints(choicePoint, out aroundPoints);
                    foreach(Point point in aroundPoints)
                    {
                        point.CalculateF(startPoint, endPoint); // 计算其预估值F
                        // 如果该点即不在openList, 也不在closeList, 就设置其父节点后, 添加到openList中
                        if(!openList.Contains(point) && !closeList.Contains(point))
                        {
                            point.Parent = choicePoint;
                            openList.Add(point);
                        }
                        // 如果该点在openList中, 带本次计算的F值更小, 说明我们从一个更快速的路径到达了它,
                        // 那么就更新其F值, 并设置其父节点为当前节点
                        if(openList.Contains(point) && point.F < openList.Find(p => p.Equals(point)).F)
                        {
                            Point oldPoint = openList.Find(p => p.Equals(point));
                            oldPoint.Parent = choicePoint;
                            oldPoint.G = point.G;   // 更新其表示实际代价的G值, 因为预估值H都是点到目标点的距离, 是不变的, 所以不用改变
                        }
                        // 如果该点在closeList中, 其本次计算的F更小, 说明我们从一个更快速的路径到达了它,
                        // 那么就更新其F值, 并设置其父节点为当前节点, 然后将其移会openList中
                        if(closeList.Contains(point) && point.F < closeList.Find(p => p.Equals(point)).F)
                        {
                            point.Parent = choicePoint;
                            // 直接删除并添加已经是新数据的点更快
                            closeList.Remove(point);
                            openList.Add(point);
                        }
                        closeList.Add(choicePoint); // 该点处理完毕, 放到closeList中
                    }
                    openList.Sort(); // 升序排序
                }
                // 寻路失败, 返回null
                return null;
            }

            /// <summary>
            /// 寻找目标点周围的点
            /// </summary>
            /// <param name="targetPoint"></param>
            /// <param name="result"></param>
            private void FindAroundPoints(Point targetPoint, out List<Point> result)
            {
                result = new List<Point>();
                // top
                if( targetPoint.Y + 1 < gridHeight)
                {
                    Point topPoint = gridDate[targetPoint.X, targetPoint.Y + 1];
                    AddCanPassPoint(topPoint, ref result);
                    if(canDiagonalMove)
                    {
                        // top-left
                        if (targetPoint.X - 1 >= 0)
                        {
                            Point topLeftPoint = gridDate[targetPoint.X - 1, targetPoint.Y + 1];
                            AddCanPassPoint(topLeftPoint, ref result);
                        }
                        // top-right
                        if (targetPoint.X + 1 < gridWidth)
                        {
                            Point topRightPoint = gridDate[targetPoint.X + 1, targetPoint.Y + 1];
                            AddCanPassPoint(topRightPoint, ref result);
                        }
                    }
                }
                // down
                if(targetPoint.Y - 1 >= 0)
                {
                    Point downPoint = gridDate[targetPoint.X, targetPoint.Y - 1];
                    AddCanPassPoint(downPoint, ref result);
                    if(canDiagonalMove)
                    {
                        // down-left
                        if (targetPoint.X - 1 >= 0)
                        {
                            Point downLeftPoint = gridDate[targetPoint.X - 1, targetPoint.Y - 1];
                            AddCanPassPoint(downLeftPoint, ref result);
                        }
                        // down-right
                        if (targetPoint.X + 1 < gridWidth)
                        {
                            Point downRightPoint = gridDate[targetPoint.X + 1, targetPoint.Y - 1];
                            AddCanPassPoint(downRightPoint, ref result);
                        }
                    }   
                }
                // left
                if(targetPoint.X - 1 >= 0)
                {
                    Point leftPoint = gridDate[targetPoint.X - 1, targetPoint.Y];
                    AddCanPassPoint(leftPoint, ref result);
                }
                // right
                if(targetPoint.X + 1 < gridWidth)
                {
                    Point rightPoint = gridDate[targetPoint.X + 1, targetPoint.Y];
                    AddCanPassPoint(rightPoint, ref result);
                }
            }

            /// <summary>
            /// 将能够通过的点添加到列表中去
            /// </summary>
            /// <param name="point"></param>
            /// <param name="resultList"></param>
            private void AddCanPassPoint(Point point, ref List<Point> resultList)
            {
                if(point.CanPass)
                {
                    resultList.Add(point);
                }
            }

        }   // end class


    } // end Astar namespace
}   // end PathSearch namespace
