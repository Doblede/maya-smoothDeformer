#include "smoothDeformer.h"

// Note: Ensure this ID is requested from Autodesk for production use to avoid conflicts.
MTypeId SmoothDeformer::id(0x000494f2);

MObject SmoothDeformer::aStrength;
MObject SmoothDeformer::aSmoothBorders;
MObject SmoothDeformer::aMaintain;
MObject SmoothDeformer::aSmoothType;
MObject SmoothDeformer::aLambda;
MObject SmoothDeformer::aMu;

SmoothDeformer::SmoothDeformer() {
    CHECK_MSTATUS(MThreadPool::init());
}

SmoothDeformer::~SmoothDeformer() {
    MThreadPool::release();
}

void* SmoothDeformer::creator() {
    return new SmoothDeformer();
}

MStatus SmoothDeformer::getInputMesh(MDataBlock& dataBlock, unsigned int geomIndex, MObject& oInputGeom) {
    MStatus status;
    
    // Using output array to prevent Maya from triggering unnecessary dirty propagation
    MArrayDataHandle hInput = dataBlock.outputArrayValue(input, &status);
    CHECK_MSTATUS(status);
    
    status = hInput.jumpToElement(geomIndex);
    if (status) {
        oInputGeom = hInput.outputValue().child(inputGeom).asMesh();
    }
    
    return status;
}

std::vector<float> SmoothDeformer::getWeightList(MDataBlock& dataBlock, unsigned int geomIndex, unsigned int numVertex) {
    std::vector<float> paintWeights(numVertex, 1.0f);
    for (unsigned int index = 0; index < numVertex; ++index) {
        paintWeights[index] = weightValue(dataBlock, geomIndex, index);
    }
    return paintWeights;
}

MStatus SmoothDeformer::deform(MDataBlock& dataBlock, MItGeometry& itGeo, const MMatrix& localToWorldMatrix, unsigned int geomIndex) {
    MStatus status;

    float envelopeValue = dataBlock.inputValue(envelope, &status).asFloat();
    CHECK_MSTATUS(status);
    if (envelopeValue <= 0.0f) {
        return MS::kSuccess; // Early exit if envelope is 0
    }

    int iterations = dataBlock.inputValue(aStrength, &status).asInt();
    CHECK_MSTATUS(status);
    if (iterations <= 0) {
        return MS::kSuccess;
    }

    float maintainValue = dataBlock.inputValue(aMaintain, &status).asFloat();
    CHECK_MSTATUS(status);

    bool smoothBorders = dataBlock.inputValue(aSmoothBorders, &status).asBool();
    CHECK_MSTATUS(status);

    short smoothType = dataBlock.inputValue(aSmoothType, &status).asShort();
    CHECK_MSTATUS(status);

    float lambdaInput = dataBlock.inputValue(aLambda, &status).asFloat();
    CHECK_MSTATUS(status);

    float mu = dataBlock.inputValue(aMu, &status).asFloat();
    CHECK_MSTATUS(status);

    MObject inputGeomObj;
    status = getInputMesh(dataBlock, geomIndex, inputGeomObj);
    CHECK_MSTATUS(status);

    TaskData taskData;
    itGeo.allPositions(taskData.points);
    itGeo.allPositions(taskData.newPoints); // Initialize newPoints to original positions
    
    taskData.envelope = envelopeValue;
    taskData.inputGeom = inputGeomObj;
    taskData.iterations = iterations;
    taskData.maintainValue = maintainValue;
    taskData.smoothBorders = smoothBorders;
    taskData.lambda = lambdaInput;
    taskData.paintWeights = getWeightList(dataBlock, geomIndex, itGeo.count());

    // Cache vertex normals
    MFnMesh fnMesh(inputGeomObj, &status);
    CHECK_MSTATUS(status);
    fnMesh.getVertexNormals(true, taskData.normals, MSpace::kTransform);

    // Evaluation loop (Type 0 = Laplacian, Type 1 = Taubin)
    for (int type = 0; type <= smoothType; type++) {
        for (int iter = 0; iter < iterations; iter++) {
            unsigned int numTasks = MThreadUtils::getNumThreads();
            ThreadData* pThreadData = createThreadData(numTasks, &taskData);
            
            MThreadPool::newParallelRegion(createTasks, (void*)pThreadData);
            
            // Swap buffers for the next iteration
            taskData.points = taskData.newPoints; 
            delete[] pThreadData;
        }
        // If Taubin, flip lambda and add mu for the shrinking correction pass
        taskData.lambda = (-1.0f * lambdaInput) + mu;
    }

    // Set final positions
    itGeo.setAllPositions(taskData.points);

    return MS::kSuccess;
}

ThreadData* SmoothDeformer::createThreadData(unsigned int numTasks, TaskData* pTaskData) {
    ThreadData* pThreadData = new ThreadData[numTasks];
    unsigned int numPoints = pTaskData->points.length();
    unsigned int taskLength = (numPoints + numTasks - 1) / numTasks;
    
    unsigned int start = 0;
    unsigned int end = taskLength;

    for (unsigned int i = 0; i < numTasks; i++) {
        if (i == numTasks - 1) {
            end = numPoints; // Ensure the last task grabs any remainder
        }
        
        pThreadData[i].start = start;
        pThreadData[i].end = end;
        pThreadData[i].numTasks = numTasks;
        pThreadData[i].pTaskData = pTaskData;

        start += taskLength;
        end += taskLength;
    }
    
    return pThreadData;
}

void SmoothDeformer::createTasks(void* pData, MThreadRootTask* pRoot) {
    ThreadData* pThreadData = (ThreadData*)pData;
    if (pThreadData) {
        int numTasks = pThreadData->numTasks;
        for (int i = 0; i < numTasks; ++i) {
            MThreadPool::createTask(threadEvaluate, (void*)&pThreadData[i], pRoot);
        }
        MThreadPool::executeAndJoin(pRoot);
    }
}

MThreadRetVal SmoothDeformer::threadEvaluate(void* pParam) {
    ThreadData* pThreadData = (ThreadData*)(pParam);
    TaskData* pTaskData = pThreadData->pTaskData;

    unsigned int start = pThreadData->start;
    unsigned int end = pThreadData->end;

    MPointArray& points = pTaskData->points;
    float envelope = pTaskData->envelope;
    float maintainValue = pTaskData->maintainValue;
    bool smoothBorders = pTaskData->smoothBorders;
    float lambda = pTaskData->lambda;

    // Local vertex iterator for this thread's chunk
    MItMeshVertex vertexIt(pTaskData->inputGeom);

    for (unsigned int index = start; index < end; ++index) {
        if (index >= points.length()) break;

        int prevPtr = 0;
        vertexIt.setIndex(index, prevPtr);

        // Respect border smoothing rule
        if (vertexIt.onBoundary() && !smoothBorders) {
            continue;
        }

        MIntArray connectedVertex;
        vertexIt.getConnectedVertices(connectedVertex);
        
        unsigned int numConnected = connectedVertex.length();
        if (numConnected == 0) continue;

        // Calculate average position of neighbors
        MPoint totalPos(0.0, 0.0, 0.0);
        for (unsigned int i = 0; i < numConnected; i++) {
            totalPos += points[connectedVertex[i]];
        }
        
        MPoint averagePoint = totalPos / static_cast<double>(numConnected);

        // Offset math correctly utilizing MVector
        MVector offsetVec = averagePoint - points[index];
        offsetVec *= (envelope * pTaskData->paintWeights[index] * lambda);

        // Normal volume preservation math
        MVector normalVec(pTaskData->normals[index]);
        double amount = offsetVec.length(); 
        
        // Final position
        MPoint newPoint = points[index] + offsetVec + (normalVec * amount * maintainValue);

        pTaskData->newPoints.set(newPoint, index);
    }
    
    return 0;
}

MStatus SmoothDeformer::initialize() {
    MFnNumericAttribute nAttr;
    MFnEnumAttribute enumAttr;

    // Smooth Algorithm Type
    aSmoothType = enumAttr.create("smoothAlgorithm", "smoothAlgorithm", 0);
    enumAttr.addField("Laplacian", 0);
    enumAttr.addField("Taubin", 1);
    enumAttr.setKeyable(false);
    enumAttr.setChannelBox(true);
    CHECK_MSTATUS(addAttribute(aSmoothType));
    CHECK_MSTATUS(attributeAffects(aSmoothType, outputGeom));

    // Strength (Iterations)
    aStrength = nAttr.create("strength", "st", MFnNumericData::kInt);
    nAttr.setKeyable(true);
    nAttr.setMin(0.0);
    nAttr.setMax(300.0);
    nAttr.setDefault(0.0);
    CHECK_MSTATUS(addAttribute(aStrength));
    CHECK_MSTATUS(attributeAffects(aStrength, outputGeom));

    // Smooth Borders Flag
    aSmoothBorders = nAttr.create("smoothBorders", "smb", MFnNumericData::kBoolean, 0);
    nAttr.setKeyable(false);
    nAttr.setChannelBox(true);
    CHECK_MSTATUS(addAttribute(aSmoothBorders));
    CHECK_MSTATUS(attributeAffects(aSmoothBorders, outputGeom));

    // Maintain Volume Bias
    aMaintain = nAttr.create("maintainVolume", "mtn", MFnNumericData::kFloat);
    nAttr.setKeyable(true);
    nAttr.setMin(0.0);
    nAttr.setMax(1.0);
    nAttr.setDefault(0.0);
    CHECK_MSTATUS(addAttribute(aMaintain));
    CHECK_MSTATUS(attributeAffects(aMaintain, outputGeom));

    // Lambda (Smoothing Factor)
    aLambda = nAttr.create("lambda", "lam", MFnNumericData::kFloat);
    nAttr.setKeyable(true);
    nAttr.setChannelBox(true);
    nAttr.setMin(0.0);
    nAttr.setMax(1.0);
    nAttr.setDefault(0.5);
    CHECK_MSTATUS(addAttribute(aLambda));
    CHECK_MSTATUS(attributeAffects(aLambda, outputGeom));

    // Mu (Taubin Shrinkage Correction)
    aMu = nAttr.create("mu", "mu", MFnNumericData::kFloat);
    nAttr.setKeyable(true);
    nAttr.setChannelBox(true);
    nAttr.setMin(0.0);
    nAttr.setMax(1.0);
    nAttr.setDefault(0.003);
    CHECK_MSTATUS(addAttribute(aMu));
    CHECK_MSTATUS(attributeAffects(aMu, outputGeom));

    // Register weight painting for the deformer
    MGlobal::executeCommand("makePaintable -attrType multiFloat -sm deformer smoothDeformer weights;");
    
    return MS::kSuccess;
}