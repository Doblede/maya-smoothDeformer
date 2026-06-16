#pragma once
#ifndef SMOOTH_DEFORMER_H
#define SMOOTH_DEFORMER_H

#include <maya/MThreadUtils.h>
#include <maya/MThreadPool.h>
#include <maya/MPxDeformerNode.h>
#include <maya/MItGeometry.h>
#include <maya/MItMeshVertex.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnMesh.h>
#include <maya/MPointArray.h>
#include <maya/MGlobal.h>
#include <maya/MTypes.h>
#include <maya/MVector.h>
#include <vector>

/**
 * @struct TaskData
 * @brief Holds the shared data required for threaded vertex smoothing evaluation.
 */
struct TaskData {
    MPointArray points;
    MPointArray newPoints;
    MFloatVectorArray normals;
    float envelope;
    int iterations;
    float maintainValue;
    bool smoothBorders;
    float lambda;
    MObject inputGeom;
    std::vector<float> paintWeights; 
};

/**
 * @struct ThreadData
 * @brief Defines the chunk of vertices a specific thread is responsible for computing.
 */
struct ThreadData {
    unsigned int start;
    unsigned int end;
    unsigned int numTasks;
    TaskData* pTaskData;
};

/**
 * @class SmoothDeformer
 * @brief A custom Maya deformer implementing Laplacian and Taubin smoothing algorithms.
 */
class SmoothDeformer : public MPxDeformerNode {
public:
    SmoothDeformer();
    virtual ~SmoothDeformer();
    
    static void* creator();
    static MStatus initialize();
    
    /**
     * @brief Retrieves the input mesh object for a given geometry index.
     */
    static MStatus getInputMesh(MDataBlock& dataBlock, unsigned int geomIndex, MObject& oInputGeom);
    
    /**
     * @brief Retrieves the painted weight values for the current geometry.
     */
    std::vector<float> getWeightList(MDataBlock& dataBlock, unsigned int geomIndex, unsigned int numVertex);
    
    virtual MStatus deform(MDataBlock& dataBlock,
                           MItGeometry& itGeo,
                           const MMatrix& localToWorldMatrix,
                           unsigned int geomIndex) override;

    // Threading execution functions
    ThreadData* createThreadData(unsigned int numTasks, TaskData* pTaskData);
    static void createTasks(void* data, MThreadRootTask* pRoot);
    static MThreadRetVal threadEvaluate(void* pParam);

    // Node ID
    static MTypeId id;

    // Attributes
    static MObject aMaintain;
    static MObject aStrength;
    static MObject aSmoothBorders;
    static MObject aSmoothType;
    static MObject aLambda;
    static MObject aMu;
};

#endif // SMOOTH_DEFORMER_H