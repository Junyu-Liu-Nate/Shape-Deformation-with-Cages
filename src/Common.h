#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Sparse>

using namespace std;
using namespace Eigen;


struct tuple_hash {
    template <class T1, class T2, class T3>
    std::size_t operator()(const std::tuple<T1, T2, T3>& tpl) const {
        auto& [first, second, third] = tpl;
        auto hash1 = std::hash<T1>{}(first);
        auto hash2 = std::hash<T2>{}(second);
        auto hash3 = std::hash<T3>{}(third);
        return ((hash1 ^ (hash2 << 1)) >> 1) ^ (hash3 << 1); // Combine hashes
    }
};

class Common
{
public:
    Common();
};

#endif // COMMON_H
