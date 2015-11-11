#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/Importer.hpp>

#include <OGRE/OgreMeshManager.h>
#include <OgreMeshSerializer.h>
#include <OgreDataStream.h>
#include <stdio.h>
#include <stdlib.h>

#include<GL/gl.h>

#include <vector>
#include <array>

#include <armadillo>

class Load_mesh{

public:


    void load(const std::string& filename){
        std::cout<< "Load Mesh" << std::endl;
        Assimp::Importer importer;
        const aiScene *scene = importer.ReadFile(filename,aiProcessPreset_TargetRealtime_Fast);//aiProcessPreset_TargetRealtime_Fast has the configs you'll need

        aiMesh *mesh = scene->mMeshes[0]; //assuming you only want the first mesh

        float *vertexArray;
        float *normalArray;
        //float *uvArray;


        int numVerts;

        numVerts = mesh->mNumFaces*3;
        arma::mat vertices(numVerts*3,3);
        arma::mat normals(numVerts*3,3);

        vertexArray = new float[numVerts*3];
        normalArray = new float[numVerts*3];
        //uvArray = new float[numVerts*2];

        for(unsigned int i=0;i<mesh->mNumFaces;i++)
        {
            const aiFace& face = mesh->mFaces[i];

            std::cout<< "num["<<i<<"]: " << face.mNumIndices << std::endl;

            for(int j=0;j<3;j++)
            {

                aiVector3D normal = mesh->mNormals[face.mIndices[j]];
                memcpy(normalArray,&normal,sizeof(float)*3);
                normalArray+=3;
                normals(i,0) = normal.x; normals(i,1) = normal.y; normals(i,2) = normal.z;

                aiVector3D pos = mesh->mVertices[face.mIndices[j]];

                vertices(i,0) = pos.x;
                vertices(i,1) = pos.y;
                vertices(i,2) = pos.z;
                memcpy(vertexArray,&pos,sizeof(float)*3);
                vertexArray+=3;
            }
        }

        vertices.save("/home/guillaume/roscode/vertices.txt", arma::raw_ascii);
        normals.save("/home/guillaume/roscode/vertices.txt", arma::raw_ascii);


        std::cout<< "numVerts: " << numVerts << std::endl;

      /*  std::size_t index = 0;
        for(std::size_t i = 0; i < numVerts*3;i++){
            vertices[i][0] = vertexArray[index];
            vertices[i][1] = vertexArray[index+1];
            vertices[i][2] = vertexArray[index+2];
            index=index+3;
            std::cout<< vertices[i][0] << "\t" << vertices[i][1] << "\t" << vertices[i][2] << std::endl;
        }*/



//        uvArray-=mesh->mNumFaces*3*2;
  /*      normalArray-=mesh->mNumFaces*3*3;
        vertexArray-=mesh->mNumFaces*3*3;


        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_NORMAL_ARRAY);
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);

        glVertexPointer(3,GL_FLOAT,0,vertexArray);
        glNormalPointer(GL_FLOAT,0,normalArray);

        glClientActiveTexture(GL_TEXTURE0_ARB);
        glTexCoordPointer(2,GL_FLOAT,0,uvArray);

        glDrawArrays(GL_TRIANGLES,0,numVerts);
        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
*/

    }

    void load_mesh_ogre(const std::string& filename){

        using namespace Ogre;

        // "source" should contain the pathname to your mesh file
        Ogre::String source;
        source = filename;

        /*
           An alternate (better) way of doing the following would be to
           use the FileStreamDataStream class, which avoids having to use
           the more esoteric "stat" struct and stdio APIs. For more, see
           http://stderr.org/doc/ogre-doc/api/classOgre_1_1FileStreamDataStream.html

           This prevents having to create and fill a MemoryDataStream instance, as
           the FileStreamDataStream can be used directly in the "stream" c'tor below.
        */
        FILE* pFile = std::fopen( source.c_str(), "rb" );
        if (!pFile)
            OGRE_EXCEPT(Exception::ERR_FILE_NOT_FOUND,"File " + source + " not found.", "OgreMeshLoaded");

        struct stat tagStat;
        stat( source.c_str(), &tagStat );
        MemoryDataStream* memstream = new MemoryDataStream(source, tagStat.st_size, true);
        std::fread( (void*)memstream->getPtr(), tagStat.st_size, 1, pFile );
        fclose( pFile );

        // give the resource a name -- it can be the full pathname if you like, since it's
        // just going to be the key in an STL associative tree container
        MeshPtr pMesh = MeshManager::getSingleton().createManual("LocalMesh",ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

        // this part does the actual load into the live Mesh object created above
        MeshSerializer meshSerializer;
        DataStreamPtr stream(memstream);
        meshSerializer.importMesh(stream, pMesh.getPointer());

        // and finally, now that we have a named Mesh resource, we can use it
        // in our createEntity() call...
       //Entity* pF35_1 = m_pSceneMgr->createEntity("LocalMesh_Ent", "LocalMesh");
    }

private:


};
