#include "cage2d.h"

class SyncCage2D : public Cage2D
{
public:
    SyncCage2D(Mode2D mode);

    void linkCage1(SyncCage2D *other);
    void linkCage2(SyncCage2D *other);
    void move(int vertex, Eigen::Vector3f pos) override;
    void moveAllAnchors(int vertex, Eigen::Vector3f pos) override;
    void moveAllAnchors(int vertex, Vector3f pos, const std::unordered_set<int>& anchors);

private:
    bool m_isSynced = false;
    SyncCage2D *m_linkedCage1 = nullptr;
    SyncCage2D *m_linkedCage2 = nullptr;

    bool isSynced();
};
