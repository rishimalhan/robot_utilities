#define UR10e_PARAMS

#include <robot_utilities/ur_kine.cpp>

#define IKFAST_HAS_LIBRARY
#include <robot_utilities/ikfast.h> // found inside share/openrave-X.Y/python/ikfast.h
using namespace ikfast;

int GetNumFreeParameters() { return 0; }
int* GetFreeParameters() { return NULL; }
int GetNumJoints() { return 6; }

int GetIkRealSize() { return sizeof(IkReal); }

int GetIkType() { return 0x67000001; }


void ComputeFk(const IkReal* jt_config, IkReal* translation, IkReal* rotation){
    double tf[16];
    URKinematics::forward(jt_config, tf);

    translation[0] = tf[3];
    translation[1] = tf[7];
    translation[2] = tf[11];

    // rotation[0] = tf[0];
    // rotation[1] = tf[1];
    // rotation[2] = tf[2];

    // rotation[3] = tf[4];
    // rotation[4] = tf[5];
    // rotation[5] = tf[6];

    // rotation[6] = tf[8];
    // rotation[7] = tf[9];
    // rotation[8] = tf[10];

    // This will give FK about tool0
    rotation[0] = -tf[1];
    rotation[1] = -tf[2];
    rotation[2] = tf[0];

    rotation[3] = -tf[5];
    rotation[4] = -tf[6];
    rotation[5] = tf[4];

    rotation[6] = -tf[9];
    rotation[7] = -tf[10];
    rotation[8] = tf[8];
}

bool ComputeIk(const IkReal* eetrans, const IkReal* eerot, const IkReal* v_free, IkSolutionList<IkReal>& solutions){
    bool status;
    double tf[16];
    tf[12] = 0;
    tf[13] = 0;
    tf[14] = 0;
    tf[15] = 1;

    // tf[3] = eetrans[0];
    // tf[7] = eetrans[1];
    // tf[11] = eetrans[2];

    // tf[0] = eerot[0];
    // tf[1] = eerot[1];
    // tf[2] = eerot[2];

    // tf[4] = eerot[3];
    // tf[5] = eerot[4];
    // tf[6] = eerot[5];

    // tf[8] = eerot[6];
    // tf[9] = eerot[7];
    // tf[10] = eerot[8];



    // Target is wrt tool0 of urdf. We want wrt ee_link for solving IK
    tf[3] = eetrans[0];
    tf[7] = eetrans[1];
    tf[11] = eetrans[2];

    tf[0] = eerot[2];
    tf[1] = -eerot[0];
    tf[2] = -eerot[1];

    tf[4] = eerot[5];
    tf[5] = -eerot[3];
    tf[6] = -eerot[4];

    tf[8] = eerot[8];
    tf[9] = -eerot[6];
    tf[10] = -eerot[7];




    double q_sols[16*6];
    int no_sols = URKinematics::inverse(tf,q_sols,0);

    if (no_sols>0)
        status = true;
    else
        return false;

    std::vector<IkSingleDOFSolutionBase<IkReal> > vinfos(6);
    std::vector<int> vfree(1);
    vfree[0] = 0;
    int ctr = 0;
    for (int i=0; i<no_sols*6; ++i){
        if (i%6==0 && i!=0){
            ctr = 0;
            solutions.AddSolution(vinfos,vfree);
        }
        vinfos[ctr].foffset = q_sols[i];
        ctr++;
    }
    solutions.AddSolution(vinfos,vfree);
    return status;
}

