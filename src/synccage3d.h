#include "cage3d.h"

class SyncCage3D : public Cage3D
{
public:
    SyncCage3D(bool useGreen);

    void linkCage(SyncCage3D *other);
    void move(int vertex, Eigen::Vector3f pos) override;
    void moveAllAnchors(int vertex, Eigen::Vector3f pos) override;

    bool test;

private:

    bool m_isSynced = false;
    SyncCage3D *m_linkedCage = nullptr;

    bool isSynced();
};

