#include "smoothDeformer.h"
#include <maya/MFnPlugin.h>

MStatus initializePlugin(MObject obj) {
    MStatus status;
    // Updated generic plugin registration
    MFnPlugin plugin(obj, "David De Juan", "1.0", "Any"); 
    
    status = plugin.registerNode(
        "smoothDeformer", 
        SmoothDeformer::id, 
        SmoothDeformer::creator, 
        SmoothDeformer::initialize, 
        MPxNode::kDeformerNode
    );
    
    CHECK_MSTATUS(status);
    return MS::kSuccess;
}

MStatus uninitializePlugin(MObject obj) {
    MStatus status;
    MFnPlugin plugin(obj);
    
    status = plugin.deregisterNode(SmoothDeformer::id);
    CHECK_MSTATUS(status);
    
    return MS::kSuccess;
}