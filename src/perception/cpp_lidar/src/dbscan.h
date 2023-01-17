#ifndef DBSCAN_H
#define DBSCAN_H

#include <cmath>
#include <vector>

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

using namespace std;

typedef struct Point_ {
    float x, y, z, i;  // X, Y, Z position
    int clusterID;     // clustered ID
} Point;

class DBSCAN {
   public:
    DBSCAN(unsigned int minPts, unsigned int maxPts, float eps, vector<Point> points) {
        m_minPoints = minPts;
        m_maxPoints = maxPts;
        m_epsilon = eps;
        m_points = points;
        m_pointSize = points.size();
    }
    ~DBSCAN() {}

    int run();
    vector<int> calculateCluster(Point point);
    int expandCluster(Point point, int clusterID);
    inline double calculateDistance(const Point& pointCore, const Point& pointTarget);
    vector<Point> getClusteredPoints(int clusterID);
    int getNumClusters();

    int getTotalPointSize() { return m_pointSize; }
    int getMinimumClusterSize() { return m_minPoints; }
    int getEpsilonSize() { return m_epsilon; }

   public:
    vector<Point> m_points;

   private:
    unsigned int m_pointSize;
    unsigned int m_minPoints;
    unsigned int m_maxPoints;
    float m_epsilon;
};

#endif  // DBSCAN_H
