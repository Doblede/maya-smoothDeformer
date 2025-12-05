# smoothDeformer
An Autodesk Maya deformer that implements the Laplacian and Taubin smoothing algorithms.
The deformer is multithreading, running in as many threads as number of cores the machine has.


The Laplacian algorithm computes, for each vertex, the average position of its connected neighbors. While this method is computationally efficient, it has a notable drawback: it does not preserve the overall volume of the mesh. To mitigate that effect a "maintain volume" attribute was added that pushes the surface along the normals.
The Taubin smoothing algorithm extends the Laplacian method by introducing a two-step filtering process that maintains volume of the mesh.

The deformer has the following attributes:
- envelope
- smooth algorithm: Laplacian or Taubin
- strengh: it is the strength of the smooth. The higher strength, the smoother the mesh will be.
- smooth borders: by default is off. By turning it on, the smooth is applied to the mesh borders.
- maintain volume: when enabled, it pushes vertices along their normals to help preserve the mesh’s overall volume.
- lambda: only used for the Taubin algorithm, it controls the smoothing step
- mu: only used for the Taubin algorithm, it controls the inflation step


To create the deformer just run
```python
cmds.deformer(type='smoothDeformer')
```