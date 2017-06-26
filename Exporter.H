/**
* /author Bret Jackson
*
* /file  Exporter.H
* /brief Exports model files using assimp
* 
*/ 

#ifndef EXPORTER_H
#define EXPORTER_H

#include <memory>
#include <string>
#include <boost/thread.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h> 
#include <assimp/DefaultLogger.hpp>
#include <assimp/LogStream.hpp>
#include <assimp/IOSystem.hpp>
#include "Artwork.H"

typedef std::shared_ptr<class Exporter> ExporterRef;

class Exporter : public std::enable_shared_from_this<Exporter>
{
public:
	Exporter();
	~Exporter();

	// Exports the non hidden marks in the artwork
	void exportArtwork(const std::string &filename, ArtworkRef artwork);

private:
	aiNode* createNode(const MarkRef mark, aiNode *pParent, aiScene* pScene, std::vector<aiMesh*> &MeshArray);
	void appendChildToParentNode(aiNode *pParent, aiNode *pChild);


	boost::thread_specific_ptr<Assimp::Exporter> _exporter;
};

#endif