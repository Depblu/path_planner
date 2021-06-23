## 修改

### 添加 JPS 替换 2d Astar
* CollisionDetection 类 添加下面函数，供JPS搜索时调用

        inline bool operator()(unsigned x, unsigned y) const

* 替换aStar函数，改为JPS调用实现

        float aStar(Node2D& start,`
            Node2D& goal,
            Node2D* nodes2D,
            int width,
            int height,
            CollisionDetection& configurationSpace,
            Visualize& visualization)
        {
          const unsigned step = 1; // 0 compresses the path as much as possible and only records waypoints.
          // Set this to 1 if you want a detailed single-step path
          // (e.g. if you plan to further mangle the path yourself),
          // or any other higher value to output every Nth position.
          // (Waypoints are always output regardless of the step size.)
        
          JPS::PathVector path; // The resulting path will go here.
          // You may also use std::vector or whatever, as long as your vector type
          // has push_back(), begin(), end(), resize() methods (same semantics as std::vector).
          // Note that the path will NOT include the starting position!
          // --> If called with start == end it will report that a path has been found,
          //     but the resulting path vector will be empty!
        
        
          // Single-call interface:
          // (Further remarks about this function can be found near the bottom of this file.)
          // Note that the path vector is NOT cleared! New path points are appended at the end.
          bool found = JPS::findPath(path, configurationSpace, start.getX(), start.getY(), goal.getX(), goal.getY(), step);
          float score = 0;
          if(found){
            for(unsigned int i = 1; i < path.size(); i++){
              Node2D n2d(path[i].x, path[i].y, 0, 0, nullptr);
              n2d.discover();
              if (Constants::visualization2D) {
                visualization.publishNode2DPoses(n2d);
              }
              const int dx = (int(path[i].x - path[i-1].x));
              const int dy = (int(path[i].y - path[i-1].y));
              //Euclidean + Chebyshev can make the search faster. Why??
              score += sqrt((float)(dx*dx + dy*dy));
              //score += dx*dx + dy*dy;
              score += JPS::Heuristic::Chebyshev(path[i], path[i-1]);
            }
          }else{
            score = 1000000.0;
          }
          //ROS_INFO("%s score:%f", __FUNCTION__, score);
          return (float)score;
        
        }

+ 上述代码有个未解之谜，就是单独使用Chebyshev或Euclidean距离时，整体搜索是速度都比较慢， 但是合并使用两种距离后，大大提速。

----
## 待解决问题
1. 1米的网格分辨率，对实际使用来说太过粗糙；初步想法是采用分层网格；3D Astar空间扩展时使用1米网格，碰撞检测采用小分辨率网格。
2. 即便原作使用轨迹平滑后，仍然不够理想（存在蛇型轨迹），此部分需要优化。
3. JPS搜索时对跳点的处理，导致轨迹在斜向再平直时存在明显转弯；要么对JPS结果进行后处理，要么通过上面的优化一并处理。
4. 转折的惩罚不够，可以考虑结合泰森多边形做cost计算。
5. grid地图4周需要添加障碍，或者在处理泰森多边形时将地图边界视为障碍，否则生成的泰森多边形有异常。
