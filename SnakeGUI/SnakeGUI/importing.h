#include <iostream>
#include <map>
#include <vector>
#include <cassert>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <Eigen/Eigen>
#define aisgl_min(x,y) (x<y?x:y)
#define aisgl_max(x,y) (y>x?y:x)
std::vector<aiVector3D> loadModel(const std::string& fileName);
std::vector<aiVector3D> InitMesh(unsigned int Index, const aiMesh* paiMesh);
