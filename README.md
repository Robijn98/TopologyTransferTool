<div align="center">

<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/NCCA/programming-project-Robijn98">
    <img src="images/wireFrame.png" alt="Logo" width="250">
  </a>
  <h3 align="center">Wrapping meshes</h3>
  

<!-- ABOUT THE PROJECT -->
## About The Project
<div align="left">


Wrapping mesh is an offline tool build that will allow you to wrap the topology of a source mesh onto a target mesh. 
The tool right now is able to copy the broad lines of a mesh, taking 8 reference points and using barycentric coordinates
to preserve the relative positions of the vertices from the source when it's wrapped to the target. 

### Built With
<div align="center">
  
[![C++](https://img.shields.io/badge/C++-00599C.svg?&logo=cplusplus&logoColor=white)](#)
[![CMake](https://img.shields.io/badge/build-CMake-blue?logo=cmake&logoColor=white)](#)
[![Linux](https://img.shields.io/badge/platform-Linux-green?logo=linux&logoColor=white)](#)
[![GTest](https://img.shields.io/badge/tests-Google%20Test-brightgreen?logo=googletest&logoColor=white)](#)
[![CGAL](https://img.shields.io/badge/CGAL-5.6.1-blue)](#)
[![NGL](https://img.shields.io/badge/uses-NGL-orange)](#)


<div align="left">
  
<!-- GETTING STARTED -->
## Getting Started
### Prerequisites

To run you will need to following 
- QT6 or QT5
- CMake 3.12
- CGAL 5.6.1
- GTest

### Installation
Clone the repo
   ```sh
   git clone https://github.com/NCCA/programming-project-Robijn98.git
   ```
<!-- USAGE EXAMPLES -->
## Usage

To use the program you can put two files in main and adjust the z en y threshold.
The z en y treshold will determine how close to the xy-plane for z en the xz-plane for y
the program will search for the middle lines, if the program doesn't run adjusting the 
tresholds might be needed.
> [!WARNING]  
> <div style="color: red;">Any meshes without vertices close to these planes won't work

<!-- ROADMAP -->
## Roadmap to the future
- [x] Use Barycentric coordinates to wrap meshes
- [x] Have color display how far the point has moved from it's orginal position
- [x] Add a viewer
- [ ] Add a laplacian smoother
- [ ] Have a UI for inputs
- [ ] Add more precision


<!-- class-diagram -->
## Class diagram
![class diagram](https://github.com/NCCA/programming-project-Robijn98/blob/main/class_diagram/class_diagram.png)


<!-- ACKNOWLEDGMENTS -->
## Acknowledgments
https://github.com/othneildrew/Best-README-Template/blob/main/README.md#roadmap


