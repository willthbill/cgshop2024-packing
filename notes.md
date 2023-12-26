# Setup
- Slurm
- docker
- git
- scip (for mip)
- cgal
- boost
- cmake

# TODO

# Notes
- remove expand arg in io
- min convex cover
- place one at a time with linear programmering. We view the existing items already placed as static and merge them with a polygon set. Then we can also smooth the sides of this polygon set to get a smaller input. We can place using the existing code and minimize y of item.
- DONEISH make the snapper work with arbitrary grid size
- use higher MIP gap in the beginning and then decrease it.
- use bounding box trick to simplify iteminitem constraints. 
- use separator lines to simplify iteminitem constraints. Supportvectormachine?
- use simplication algorithm on configuration spaces (its dynamically between full config space and bounding box basically)
- play with gurobi parameters
- why are we using the snapper right now in the iteminitem constraint? it might be floating point right?
- skaffe adgang til itu server
- gøre ref x og ref y til floats variables. why?

- place one piece at a time and minimize y, but still keep all items

- value / area of CH

- try to use the fast integer approximation algorithm where we also add 2,3 or something like that in the correct direction
- research what makes the integer approximation algorithm fail?
- allow some pieces to move around and others to be part of the big piece. for example most recent pieces can move