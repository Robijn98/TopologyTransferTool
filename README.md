
Face wrapping

https://graphics.pixar.com/library/WrapperSIGGRAPH2019/paper.pdf
(paper has been looked at by Jian, he thinks the math isn't realistic so I've started looking at another way to achieve what I set out to do, as described in the plan)

I want to write a piece of code that takes two meshes as inputs, one will be the source model and one the target. 
The source model will wrap around the target while accurately resembling the patch layout of the source mesh.
Curves will be placed on both models to guide the wrap optimization.

plan:
- Map out the source mesh, using barycentric coordinates to map all given points relative to the guidance curves
- Discretize all given curves on source model (pre determinted) 
- project point obtained during discretizing from source curve to target curve along the vector direction
- using the source curves to define the fixed boundary points, we can use the geometry of the target mesh and the barycentric information form the source to calculate discrete laplacians (e.g. contangent weights for surface meshes) 
- solve for interior points 
- validate that the resulting topology respects the initial mapping and constraints




Once you have wrapped the face you can use maya or a different software to blend from one rig to another and use previously created blendshapes on the new mesh. 

![class diagram](https://github.com/NCCA/programming-project-Robijn98/blob/main/class_diagram/class_diagram.png)