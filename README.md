
Face wrapping

https://graphics.pixar.com/library/WrapperSIGGRAPH2019/paper.pdf

I want to write a piece of code that takes two meshes as inputs, one will be the source model and one the target. 
The source model will wrap around the target while accurately resembling the patch layout of the source mesh.
Curves will be placed on both models to guide the wrap optimization. (maybe I should start with points first and see if I can build out to curves as points are easier to interpret?)

Once you have wrapped the face you can use maya or a different software to blend from one rig to another and use previously created blendshapes on the new mesh. 
