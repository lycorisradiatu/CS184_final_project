#ifndef STUDENT_CODE_H
#define STUDENT_CODE_H

#include "halfEdgeMesh.h"
#include "bezierPatch.h"
#include "bezierCurve.h"

using namespace std;

namespace CGL {

  class MeshResampler{

  public:

    MeshResampler(){};
    ~MeshResampler(){}

    void upsample(HalfedgeMesh& mesh);
    double get_s_j(int j, int K);
    void upsample_butterfly_scheme(HalfedgeMesh& mesh);
  };
}

#endif // STUDENT_CODE_H
