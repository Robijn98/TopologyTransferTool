
Face wrapping

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



[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![Unlicense License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]


[forks-shield]: https://img.shields.io/github/forks/othneildrew/Best-README-Template.svg?style=for-the-badge
[forks-url]:https://github.com/NCCA/programming-project-Robijn98/forks
[stars-shield]: https://img.shields.io/github/stars/othneildrew/Best-README-Template.svg?style=for-the-badge
[stars-url]: https://github.com/NCCA/programming-project-Robijn98/stargazers
[issues-shield]: https://img.shields.io/github/issues/othneildrew/Best-README-Template.svg?style=for-the-badge
[issues-url]: https://github.com/NCCA/programming-project-Robijn98/issues
[license-shield]: https://img.shields.io/github/license/othneildrew/Best-README-Template.svg?style=for-the-badge
[license-url]: https://github.com/othneildrew/Best-README-Template/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/in/robin-van-den-eerenbeemd-23494a172/

<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/NCCA/programming-project-Robijn98">
    <img src="images/wireFrame.png" alt="Logo" width="80" height="80">
  </a>
  <h3 align="center">Wrapping meshes</h3>
  

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>


<!-- ABOUT THE PROJECT -->
## About The Project
Wrapping mesh is an offline tool build that will allow you to wrap the topology of a source mesh onto a target mesh. 
The tool right now is able to copy the broad lines of a mesh, taking 8 reference points and using barycentric coordinates
to preserve the relative positions of the vertices from the source when it's wrapped to the target. 

### Built With
[![C++][https://img.shields.io/badge/C++-00599C.svg?&style=for-the-badge&logo=cplusplus&logoColor=white]]
[![CMake][https://img.shields.io/badge/build-CMake-blue?logo=cmake&logoColor=white]]
[![Linux][https://img.shields.io/badge/platform-Linux-green?logo=linux&logoColor=white]]
[![CGAL][https://img.shields.io/badge/CGAL-5.5.2-blue]]
[![NGL][https://img.shields.io/badge/uses-NGL-orange]]

<!-- GETTING STARTED -->
## Getting Started
### Prerequisites
