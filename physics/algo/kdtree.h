#ifndef KDTREE_H
#define KDTREE_H

#include <vector>
#include <QVector>
#include <algorithm>
#include <limits>
#include <QtAlgorithms>
#include <QDebug>

template <class PointT>
class KDTree
{
public:
    /** @brief The constructors.
     */
    KDTree();

    KDTree(QVector<PointT>& cloud);

    /** @brief The destructors.
     */
    ~KDTree();

    bool isEmpty()
    {
        return (root == nullptr);
    }

    /** @brief Set or re-set kd tree.
     *  @param[in] cloud the const boost shared pointer to a PointCloud message
     */
    void setInputCloud(const QVector<PointT>& cloud);

    /** @brief Clears k-d tree.
     */
    void clear();

    /** @brief Searches nearest neighbors.
     *  @param[in] point the given query point
     *  @param[out] minDist the minimum distance between the queried point and othor points in the point cloud
     *  @return index of nearest neighbors
     */
    int nnSearch(const PointT &point, float* minDist = nullptr) const;

    /** @brief Search for the k-nearest neighbors for the given query point.
     *  @param[in] point the given query point
     *  @param[in] k the number of neighbors to search for
     *  @param[out] k_indices the resultant indices of the neighboring points (must be
     * resized to k a priori!)
     *  @param[out] k_sqr_distances the resultant squared distances to the neighboring
     * points (must be resized to k a priori!)
     *  @return number of neighbors found
     */
    int KnnSearch(const PointT &point, int k, QVector<int> &k_indices,
                  QVector<float> &k_sqr_distances) const;

    /** @brief Search for all the nearest neighbors of the query point in a given radius.
     *  @param[in] point the given query point
     *  @param[in] radius the radius of the sphere bounding all of queried point's neighbors
     *  @param[out] k_indices the resultant indices of the neighboring points (must be resized
     * to k a priori!)
     *  @param[out] k_sqr_distances the resultant squared distances to the neighboring points
     * (must be resized to k a priori!)
     *  @param[in] maxnn if given, bounds the maximum returned neighbors to this value. If \a
     * max_nn is set to 0 or to a number higher than the number of points in the input cloud,
     * all neighbors in \a radius will be returned.
     *  @return number of neighbors found
     */
    int RadiusSearch(const PointT &point, float radius, QVector<int> &k_indices,
                     QVector<float> &k_sqr_distances, int max_nn = 0) const;

    /** @brief Search for all the nearest neighbors of the query point in a given plane.
     *  @param[in] point the given query point
     *  @param[in] normal the normal of the plane given
     *  @param[in] thickness the thickness of the plane given
     *  @param[in] radius the radius of the plane bounding all of queried point's neighbors
     *  @param[out] k_indices the resultant indices of the neighboring points (must be resized
     * to k a priori!)
     *  @param[in] maxnn if given, bounds the maximum returned neighbors to this value. If \a
     * max_nn is set to 0 or to a number higher than the number of points in the input cloud,
     * all neighbors in \a radius will be returned.
     *  @return number of neighbors found
     */
    int PlaneSearch(const PointT &point, PointT normal, float thickness, float radius, QVector<int> &k_indices, QVector<float> &k_sqr_distances, int max_nn = 0) const;

private:

    /** @brief k-d tree node.
     */
    struct Node
    {
        int idx;       //!< index to the original point
        Node* next[2]; //!< pointers to the child nodes
        int axis;      //!< dimension's axis

        Node() : idx(-1), axis(-1) { next[0] = next[1] = nullptr; }
    };

    /** @brief Bounded priority queue.
     */
    template <class T, class Compare = std::less<T>>
    class BoundedPriorityQueue
    {
    public:

        BoundedPriorityQueue() = delete;
        BoundedPriorityQueue(size_t bound) : bound_(bound) { elements_.reserve(bound + 1); };

        void push(const T& val)
        {
            auto it = std::find_if(std::begin(elements_), std::end(elements_),
                [&](const T& element){ return Compare()(val, element); });
            elements_.insert(it, val);

            if (elements_.size() > bound_)
                elements_.resize(bound_);
        }

        const T& back() const { return elements_.back(); };
        const T& operator[](size_t index) const { return elements_[index]; }
        size_t size() const { return elements_.size(); }

    private:
        size_t bound_;
        std::vector<T> elements_;
    };

    /** @brief Priority queue of <distance, index> pair.
    */
    using KnnQueue = BoundedPriorityQueue<std::pair<float, int>>;
    using RadiusQueue = BoundedPriorityQueue<std::pair<float, int>>;
    using PlaneQueue = BoundedPriorityQueue<std::pair<float, int>>;

    /** @brief The input point cloud dataset containing the points we need to use. */
    QVector<PointT> input;
    /** @brief A Pointer to the vector of point cloud indices to use. */
    QVector<int> indices;
    /** @brief The number of point dimensions per point. */
    int dim;
    /** @brief The root node. */
    Node* root;


    /** @brief Builds k-d tree recursively.
     *  @param indices[in] the pointer to the data of point cloud indices to use
     *  @param npoints[in] the number of point cloud to use
     *  @param depth[in]
     *  @return the root node
    */
    Node* buildRecursive(int* indices, int npoints, int depth);

    /** @brief Clears k-d tree recursively.
     *  @param[in] node Currently operating node
     */
    void clearRecursive(Node* node);

    /** @brief Calculate the distance between two points.
     */
    static double distance(const PointT& p, const PointT& q)
    {
        double dist = 0;
        for (size_t i = 0; i < 3; i++)
            dist += (p[i] - q[i]) * (p[i] - q[i]);
        return sqrt(dist);
    }

    /** @brief Calculate the Dot production between two points.
     */
    static double Dot(const PointT& p, const PointT& q)
    {
        double dist = 0;
        for (size_t i = 0; i < 3; i++)
            dist += p[i] * q[i];
        return dist;
    }

    /** @brief Calculate the Cross production between two points.
     */
    static PointT Cross(const PointT& p, const PointT& q)
    {
        PointT dist;
        dist[0] = p[1] * q[2] - q[1] * p[2];
        dist[1] = p[0] * q[2] - q[0] * p[2];
        dist[2] = p[0] * q[1] - q[0] * p[1];
        return dist;
    }

    /** @brief Calculate the Cross production between two points.
     */
    static PointT Normalized(const PointT& p)
    {
        double d = distance(p, p);
        PointT q = d * p;
        return q;
    }

    /** @brief Search for the nearest neighbor recursively.
     *  @param point[in] the given query point
     *  @param node[in] Node currently retrieved
     *  @param guess[out]
     *  @param minDist[in] the minimum distance between the queried point and othor points in the point cloud
     */
    void nnSearchRecursive(const PointT &point, const Node* node, int* guess, float* minDist) const;

    /** @brief Searches k-nearest neighbors recursively.
     */
    void knnSearchRecursive(const PointT& point, const Node* node, KnnQueue& queue, int k) const;

    /** @brief Searches radius-nearest neighbors recursively.
     */
    void RadiusSearchRecursive(const PointT& point, const Node* node, RadiusQueue& queue, float radius) const;

    /** @brief Searched plane-nearest neighbors recursively.
     */
    void PlaneSearchRecursive(const PointT& point, const Node* node, RadiusQueue& queue, PointT normal, float thickness, float radius) const;
};

#endif // KDTREE_H
