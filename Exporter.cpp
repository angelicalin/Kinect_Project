/**
* /author Bret Jackson
*
* /file  Exporter.cpp
* /brief Exports model files using assimp
* 
*/ 

#include "Exporter.H"

Exporter::Exporter()
{
	//TODO not entirely sure this is threadsafe, although assimp says the library is as long as you have separate importer objects
	Assimp::Logger::LogSeverity severity = Assimp::Logger::NORMAL;
	// Create a logger instance for Console Output
	Assimp::DefaultLogger::create("",severity, aiDefaultLogStream_STDOUT);
}

Exporter::~Exporter()
{
	// Kill it after the work is done
	Assimp::DefaultLogger::kill();
}

void Exporter::exportArtwork(const std::string &filename, ArtworkRef artwork)
{
	if (_exporter.get() == nullptr) {
		_exporter.reset(new Assimp::Exporter());
	}

	string formatId;
	if (boost::algorithm::ends_with(filename, "dae")) {
		formatId = "collada";
	}
	else if( boost::algorithm::ends_with(filename, "obj")) {
		formatId = "obj";
	}
	else if( boost::algorithm::ends_with(filename, "stl")) {
		formatId = "stl";
	}
	else if( boost::algorithm::ends_with(filename, "ply")) {
		formatId = "ply";
	}
	else {
		std::cout<<"Unable to determine export file format. Filename should end in .dae, .obj, .stl, or .ply"<<std::endl;
		return;
	}


	// Create an assimp scene object to hold the meshes.
	aiScene* scene = new aiScene();
	// Create the root node of the scene
	scene->mRootNode = new aiNode;
	
	std::vector<aiMesh*> MeshArray;
	std::vector<MarkRef> marks = artwork->getVisibleArtMarks();

	for (int i=0; i < marks.size(); i++) {
		createNode(marks[i], scene->mRootNode, scene, MeshArray);
	}

	// Create mesh pointer buffer for this scene
	if (scene->mNumMeshes > 0) {
		scene->mMeshes = new aiMesh*[MeshArray.size()];
		for (size_t index =0; index < MeshArray.size(); index++)
		{
			scene->mMeshes[index] = MeshArray[index];
		}
	}
	
	// Just make everything have the same material
	scene->mNumMaterials = 1;
	scene->mMaterials = new aiMaterial*[1];
	aiMaterial* mat = new aiMaterial;
	aiString matName("defaultMaterial");
	mat->AddProperty(&matName, AI_MATKEY_NAME );
	int sm = aiShadingMode_Phong;
	mat->AddProperty<int>( &sm, 1, AI_MATKEY_SHADING_MODEL);

	aiColor3D diffuse(0.7, 0.7, 0.7);
	aiColor3D ambient(0.2, 0.2, 0.2);
	aiColor3D specular(1.0, 1.0, 1.0);
	float shineness = 0.5;
	float alpha = 1.0;
	mat->AddProperty( &ambient, 1, AI_MATKEY_COLOR_AMBIENT );
	mat->AddProperty( &diffuse, 1, AI_MATKEY_COLOR_DIFFUSE );
	mat->AddProperty( &specular, 1, AI_MATKEY_COLOR_SPECULAR );
	mat->AddProperty( &shineness, 1, AI_MATKEY_SHININESS );
	mat->AddProperty( &alpha, 1, AI_MATKEY_OPACITY );
	scene->mMaterials[0] = mat;


	_exporter->Export(scene, formatId, filename, 0);

	delete scene;
}

aiNode* Exporter::createNode(const MarkRef mark, aiNode *pParent, aiScene* pScene, std::vector<aiMesh*> &MeshArray)
{
	assert( NULL != mark.get() );
	
	// Store older mesh size to be able to computes mesh offsets for new mesh instances
	const size_t oldMeshSize = MeshArray.size();
	aiNode *pNode = new aiNode;

	pNode->mName = mark->getName();
	
	// If we have a parent node, store it
	if (pParent != NULL) {
		appendChildToParentNode(pParent, pNode);
	}

	aiMesh *pMesh = new aiMesh;

	std::vector<GPUMesh::Vertex> vertices;
	std::vector<std::vector<int> > indicesByFace;
	mark->getMeshForExport(vertices, indicesByFace);

	pMesh->mName = mark->getName();

	pMesh->mNumFaces = indicesByFace.size();
	if (pMesh->mNumFaces > 0)	{
		pMesh->mFaces = new aiFace[pMesh->mNumFaces];

		for(int i=0; i < indicesByFace.size(); i++) {
			aiFace *pFace = &pMesh->mFaces[i];
			pFace->mNumIndices = 3; // We only handle triangulated meshes
			pFace->mIndices = new unsigned int[3];			
		}
	}

	pMesh->mNumVertices = indicesByFace.size()*3;

	if (pMesh->mNumVertices > 0) {
		pMesh->mVertices = new aiVector3D[pMesh->mNumVertices];
		pMesh->mNormals = new aiVector3D[pMesh->mNumVertices];
		pMesh->mNumUVComponents[0] = 2;
		pMesh->mTextureCoords[0] = new aiVector3D[pMesh->mNumVertices];

		// Copy vertices, normals and textures into aiMesh instance
		for(int i=0; i < vertices.size(); i++) {
			glm::dvec3 pt = vertices[i].position;
			pMesh->mVertices[i].x = pt.x;
			pMesh->mVertices[i].y = pt.y;
			pMesh->mVertices[i].z = pt.z;

			glm::dvec3 normal = vertices[i].normal;
			pMesh->mNormals[i].x = normal.x;
			pMesh->mNormals[i].y = normal.y;
			pMesh->mNormals[i].z = normal.z;

			pMesh->mTextureCoords[0][i].x = vertices[i].texCoord0.x;
			pMesh->mTextureCoords[0][i].y = vertices[i].texCoord0.y;
			pMesh->mTextureCoords[0][i].z = 0.0;
		}

		for(int i=0; i < indicesByFace.size(); i++) {
			aiFace *pDestFace = &pMesh->mFaces[i];
			for(int j=0; j < 3; j++) {
				pDestFace->mIndices[j] = indicesByFace[i][j];
			}
		}

		MeshArray.push_back( pMesh );
	}
	else
	{
		delete pMesh;
	}
	
	// Set mesh instances into scene- and node-instances
	const size_t meshSizeDiff = MeshArray.size()- oldMeshSize;
	if ( meshSizeDiff > 0 )
	{
		pNode->mMeshes = new unsigned int[ meshSizeDiff ];
		pNode->mNumMeshes = static_cast<unsigned int>( meshSizeDiff );
		size_t index = 0;
		for (size_t i = oldMeshSize; i < MeshArray.size(); i++)
		{
			pNode->mMeshes[ index ] = pScene->mNumMeshes;
			pScene->mNumMeshes++;
			index++;
		}
	}
	
	return pNode;
}

//	Appends this node to the parent node
void Exporter::appendChildToParentNode(aiNode *pParent, aiNode *pChild)
{
	// Checking preconditions
	assert( NULL != pParent );
	assert( NULL != pChild );

	// Assign parent to child
	pChild->mParent = pParent;
	size_t sNumChildren = 0;
	(void)sNumChildren; // remove warning on release build
	
	// If already children was assigned to the parent node, store them in a 
	std::vector<aiNode*> temp;
	if (pParent->mChildren != NULL)
	{
		sNumChildren = pParent->mNumChildren;
		assert( 0 != sNumChildren );
		for (size_t index = 0; index < pParent->mNumChildren; index++)
		{
			temp.push_back(pParent->mChildren [ index ] );
		}
		delete [] pParent->mChildren;
	}
	
	// Copy node instances into parent node
	pParent->mNumChildren++;
	pParent->mChildren = new aiNode*[ pParent->mNumChildren ];
	for (size_t index = 0; index < pParent->mNumChildren-1; index++)
	{
		pParent->mChildren[ index ] = temp [ index ];
	}
	pParent->mChildren[ pParent->mNumChildren-1 ] = pChild;
}