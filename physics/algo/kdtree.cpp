#include "kdtree.h"

template <class PointT>
KDTree<PointT>::KDTree()
    : root(nullptr)
{

}

template <class PointT>
KDTree<PointT>::KDTree(QVector<PointT>& cloud)
    : root(nullptr)
{
    setInputCloud(cloud);
}

template <class PointT>
KDTree<PointT>::~KDTree<PointT>()
{
    clear();
}

template <class PointT> void
KDTree<PointT>::clear()
{
    clearRecursive(root);
    input.clear();
    indices.clear();
}

template <class PointT> void
KDTree<PointT>::setInputCloud(const QVector<PointT>& cloud)
{
    clear();   // Perform an automatic cleanup of structures
    dim = 3;
    for (int i = 0; i < cloud.size(); i++)
    {
        input.push_back( cloud[i]);
        indices.push_back(i);
    }

    if (input.empty())
    {
        qDebug() << "[KDTree::setInputCloud] Invalid input!\n" << endl;
        return;
    }

    root = buildRecursive(indices.data(), indices.size(), 0);
}

template <class PointT> int
KDTree<PointT>::nnSearch(const PointT &point, float* minDist) const
{
    int guess;
    float _minDist = std::numeric_limits<double>::max();
    nnSearchRecursive(point, root, &guess, &_minDist);

    if (minDist)
        *minDist = _minDist;

    return guess;
}

template <class PointT> int
KDTree<PointT>::KnnSearch(const PointT &point, int k, QVector<int> &k_indices,
                          QVector<float> &k_sqr_distances) const
{
    if(k > input.size())
        k = input.size();

    k_indices.resize (k);
    k_sqr_distances.resize (k);

    if (k==0)
        return 0;

    KnnQueue queue(k);
    knnSearchRecursive(point, root, queue, k);

    for (int i = 0; i < k; i++)
    {
        k_sqr_distances[i] = queue[i].first;
        k_indices[i] = queue[i].second;
    }

    return k;
}

template <class PointT> int
KDTree<PointT>::RadiusSearch(const PointT &point, float radius, QVector<int> &k_indices,
                             QVector<float> &k_sqr_distances, int max_nn) const
{
    // Has max_nn been set properly?
    if (max_nn == 0 || max_nn > input.size())
        max_nn = input.size();

    RadiusQueue queue(max_nn);

    RadiusSearchRecursive(point, root, queue, radius);

    k_sqr_distances.clear();
    k_indices.clear();

    for (int i = 0; i < queue.size(); i++)
    {
        k_sqr_distances.push_back(queue[i].first);
        k_indices.push_back(queue[i].second);
    }

    return queue.size();
}


template <class PointT> int
KDTree<PointT>::PlaneSearch(const PointT &point, PointT normal, float thickness, float radius, QVector<int> &k_indices, QVector<float> &k_sqr_distances, int max_nn) const
{
    // Has max_nn been set properly?
    if (max_nn == 0 || max_nn > input.size())
        max_nn = input.size();

    PlaneQueue queue(max_nn);

    PlaneSearchRecursive(point, root, queue, normal, thickness, radius);

    k_sqr_distances.clear();
    k_indices.clear();

    for (int i = 0; i < queue.size(); i++)
    {
        k_sqr_distances.push_back(queue[i].first);
        k_indices.push_back(queue[i].second);
    }

    return queue.size();
}






template <class PointT> typename KDTree<PointT>::Node*
KDTree<PointT>::buildRecursive(int* indices, int npoints, int depth)
{
    if (npoints <= 0)
        return nullptr;

    const int axis = depth % dim;
    const int mid = (npoints - 1) / 2;

    // Get the median point's index.
    std::nth_element(indices, indices + mid, indices + npoints, [&](int lhs, int rhs)
    {
        return input[lhs][axis] < input[rhs][axis];
    });

    Node* node = new Node;
    node->idx = indices[mid];

    node->axis = axis;

    node->next[0] = buildRecursive(indices, mid, depth + 1);
    node->next[1] = buildRecursive(indices + mid + 1, npoints - mid - 1, depth + 1);

    return node;
}

template <class PointT> void
KDTree<PointT>::clearRecursive(Node* node)
{
    if (node == nullptr)
        return;

    if (node->next[0])
        clearRecursive(node->next[0]);

    if (node->next[1])
        clearRecursive(node->next[1]);

    delete node;
}

template <class PointT> void
KDTree<PointT>::nnSearchRecursive(const PointT &point, const Node* node, int* guess, float* minDist) const
{
    if (node == nullptr)
        return;

    const PointT& test = input[node->idx];
    float dist = distance(point, test);

    if (dist < *minDist)
    {
        *minDist = dist;
        *guess = node->idx;
    }

    const int axis = node->axis;
    const int dir = point[axis] < test[axis] ? 0 : 1;
    nnSearchRecursive(point, node->next[dir], guess, minDist);

    // Determine whether the minimum distance after recursion is optimal.
    const float diff = fabs(point[axis] - test[axis]);
    if (diff < *minDist)
        // the minimun distance is non-optimal, so search another direction.
        nnSearchRecursive(point, node->next[!dir], guess, minDist);
}

template <class PointT> void
KDTree<PointT>::knnSearchRecursive(const PointT& point, const Node* node, KnnQueue& queue, int k) const
{
    if (node == nullptr)
        return;

    const PointT& test = input[node->idx];
    const double dist = distance(point, test);

    queue.push(std::make_pair(dist, node->idx));

    const int axis = node->axis;
    const int dir = point[axis] < test[axis] ? 0 : 1;
    knnSearchRecursive(point, node->next[dir], queue, k);

    // Determine whether the minimum distance after recursion is optimal
    // or whether there are k nearest neighbors are found.
    const float diff = fabs(point[axis] - test[axis]);
    if (queue.size() < k || diff < queue.back().first)
        // the minimun distance is non-optimal or the number of nearest neighbors
        // found is less than k, so search another direction.
        knnSearchRecursive(point, node->next[!dir], queue, k);
}

template <class PointT> void
KDTree<PointT>::RadiusSearchRecursive(const PointT& point, const Node* node, RadiusQueue &queue,  float radius) const
{
    if (node == nullptr)
        return;

    const PointT& test = input[node->idx];
    const float dist = distance(point, test);

    if(dist < radius)
    {
        queue.push(std::make_pair(dist, node->idx));
    }

    const int axis = node->axis;
    const int dir = point[axis] < test[axis] ? 0 : 1;
    RadiusSearchRecursive(point, node->next[dir], queue, radius);

    const float diff = fabs(point[axis] - test[axis]);
    if (diff < radius)
        RadiusSearchRecursive(point, node->next[!dir], queue, radius);
}

template <class PointT> void
KDTree<PointT>::PlaneSearchRecursive(const PointT& point, const Node* node, RadiusQueue& queue, PointT normal, float thickness, float radius) const
{
    if (node == nullptr)
        return;

    const PointT& vector = input[node->idx] - point;
    const PointT& test = input[node->idx];
    const float dist1 = distance(point, test);
    const float dist2 = fabs(Dot(normal, vector));

    if (dist1 < radius && dist2 < thickness)
        queue.push(std::make_pair(dist1, node->idx));

    const int axis = node->axis;
    const int dir = point[axis] < test[axis] ? 0 : 1;
    PlaneSearchRecursive(point, node->next[dir], queue, normal, thickness, radius);

    const float diff = fabs(point[axis] - test[axis]);
    if (diff < radius)
        PlaneSearchRecursive(point, node->next[!dir], queue, normal, thickness, radius);
}
