
#include "importing.h"
using namespace Eigen;
/* the global Assimp scene object */
//const struct aiScene* scene = NULL;

/* current rotation angle */
static float angle = 0.f;


std::vector<aiVector3D> loadModel(const std::string& fileName) {
	Assimp::Importer importer;

	const aiScene* pScene = importer.ReadFile(fileName.c_str(),
		aiProcess_Triangulate |
		aiProcess_GenSmoothNormals |
		aiProcess_FlipUVs);
	//std::vector<aiVector3D> vertices;

	if (pScene) {
		//const aiMesh* model = pScene->mMeshes[0];

		// Initialize the meshes in the scene one by one
		//for (unsigned int i = 0; i < pScene->mNumMeshes; i++) {
			const aiMesh* paiMesh = pScene->mMeshes[0];
			return InitMesh(0, paiMesh);
		//}
	}
	else {
		printf("Error parsing '%s': '%s'\n", fileName.c_str(), importer.GetErrorString());
	}
}

std::vector<aiVector3D> InitMesh(unsigned int Index, const aiMesh* paiMesh)
{

	int numVert = paiMesh->mNumVertices;
	std::vector<aiVector3D> vertices;
	std::vector<aiVector3D> normals;
	std::vector<const aiVector3D*> texcoord;
	std::vector<int> indices;

	const aiVector3D aiZeroVector(0.0f, 0.0f, 0.0f);
	for (unsigned int i = 0; i < numVert; i++)
	{
		aiVector3D pPos = (paiMesh->mVertices[i]);
		aiVector3D pNormal = (paiMesh->mNormals[i]);
		const aiVector3D* pTexCoord = paiMesh->HasTextureCoords(0) ? &(paiMesh->mTextureCoords[0][i]) : &aiZeroVector;

		vertices.push_back(pPos);
		normals.push_back(pNormal);
		texcoord.push_back(pTexCoord);
		//	Vertex vert(Vector3f(pPos->x, pPos->y, pPos->z),
		//	Vector2f(pTexCoord->x, pTexCoord->y),
		//Vector3f(pNormal->x, pNormal->y, pNormal->z));
	}
	for (unsigned int i = 0; i < paiMesh->mNumFaces; i++)
	{
		const aiFace& face = paiMesh->mFaces[i];
		assert(face.mNumIndices == 3);
		indices.push_back(face.mIndices[0]);
		indices.push_back(face.mIndices[1]);
		indices.push_back(face.mIndices[2]);
	}
	return vertices;
}

/*
void InitMesh(std::vector<aiVector3D> vertices, std::vector<aiVector3D> normals, std::vector<const aiVector3D*>, int vertSize, int* indices, int indexSize, bool calcNormals)
{
	m_meshData = new MeshData(indexSize);

	if (calcNormals)
		this->CalcNormals(vertices, vertSize, indices, indexSize);

	glBindBuffer(GL_ARRAY_BUFFER, m_meshData->GetVBO());
	glBufferData(GL_ARRAY_BUFFER, vertSize * sizeof(Vertex), vertices, GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_meshData->GetIBO());
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexSize * sizeof(int), indices, GL_STATIC_DRAW);
}
*/


/*
#include <stdio.h>
#include <fstream>
#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flag




// include GLEW to access OpenGL 3.3 functions
#include <GL/GLU.h>

// GLUT is the toolkit to interface with the OS
#include <GL/freeglut.h>

// auxiliary C file to read the shader text files
//#include "textfile.h"

// assimp include files. These three are usually needed.
#include "assimp/Importer.hpp"	//OO version Header!
#include "assimp/PostProcess.h"
#include "assimp/Scene.h"


#include <math.h>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include<vector>

bool Import3DFromFile(const std::string& pFile)
{

//check if file exists
std::ifstream fin(pFile.c_str());
if (!fin.fail()) {
fin.close();
}
else {
printf("Couldn't open file: %s\n", pFile.c_str());
printf("%s\n", importer.GetErrorString());
return false;
}

scene = importer.ReadFile(pFile, aiProcessPreset_TargetRealtime_Quality);

// If the import failed, report it
if (!scene)
{
printf("%s\n", importer.GetErrorString());
return false;
}

// Now we can access the file's contents.
printf("Import of scene %s succeeded.", pFile.c_str());

aiVector3D scene_min, scene_max, scene_center;
get_bounding_box(&scene_min, &scene_max);
float tmp;
tmp = scene_max.x - scene_min.x;
tmp = scene_max.y - scene_min.y > tmp ? scene_max.y - scene_min.y : tmp;
tmp = scene_max.z - scene_min.z > tmp ? scene_max.z - scene_min.z : tmp;
scaleFactor = 1.f / tmp;

// We're done. Everything will be cleaned up by the importer destructor
return true;
}

*/