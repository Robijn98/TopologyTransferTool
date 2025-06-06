#include "NGLScene.h"
#include "mesh.h"
#include <QMouseEvent>
#include <QGuiApplication>
#include <QFont>
#include <QFileDialog>
#include <array>
#include <ngl/Transformation.h>
#include <ngl/NGLInit.h>
#include <ngl/VAOPrimitives.h>
#include <ngl/ShaderLib.h>
#include <iostream>
#include <ngl/VAOFactory.h>
#include <ngl/SimpleIndexVAO.h>


NGLScene::NGLScene()
{
  setTitle("wrapper viewer");
}

NGLScene::~NGLScene()
{
  std::cout << "Shutting down NGL, removing VAO's and Shaders\n";
  m_vao->removeVAO();
}

void NGLScene::resizeGL(int _w, int _h)
{
  m_project = ngl::perspective(45.0f, static_cast<float>(_w) / _h, 0.05f, 350.0f);
  m_win.width = static_cast<int>(_w * devicePixelRatio());
  m_win.height = static_cast<int>(_h * devicePixelRatio());
}

void NGLScene::initializeGL()
{
  ngl::NGLInit::initialize();

  glClearColor(0.4f, 0.4f, 0.4f, 1.0f); // Grey Background
  // enable depth testing for drawing

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_MULTISAMPLE);
  // Now we will create a basic Camera from the graphics library
  // This is a static camera so it only needs to be set once
  // First create Values for the camera position
  ngl::Vec3 from(0, 1, 15);
  ngl::Vec3 to(0, 0, 0);
  ngl::Vec3 up(0, 1, 0);

  m_view = ngl::lookAt(from, to, up);
  // set the shape using FOV 45 Aspect Ratio based on Width and Height
  // The final two are near and far clipping planes of 0.5 and 10
  m_project = ngl::perspective(45, 720.0f / 576.0f, 0.001f, 150.0f);

  constexpr auto ColourShader = "ColourShader";
  constexpr auto ColourVertex = "ColourVertex";
  constexpr auto ColourFragment = "ColourFragment";
  ngl::ShaderLib::createShaderProgram(ColourShader);

  ngl::ShaderLib::attachShader(ColourVertex, ngl::ShaderType::VERTEX);
  ngl::ShaderLib::attachShader(ColourFragment, ngl::ShaderType::FRAGMENT);
  ngl::ShaderLib::loadShaderSource(ColourVertex, "shaders/ColourVertex.glsl");
  ngl::ShaderLib::loadShaderSource(ColourFragment, "shaders/ColourFragment.glsl");

  ngl::ShaderLib::compileShader(ColourVertex);
  ngl::ShaderLib::compileShader(ColourFragment);
  ngl::ShaderLib::attachShaderToProgram(ColourShader, ColourVertex);
  ngl::ShaderLib::attachShaderToProgram(ColourShader, ColourFragment);

  ngl::ShaderLib::linkProgramObject(ColourShader);
  ngl::ShaderLib::use(ColourShader);

  buildVAO();
  ngl::VAOFactory::listCreators();
}

void NGLScene::buildVAO()
{
  Mesh mesh;
  Polygon_mesh polygonSource;
  mesh.loadMesh("files/output.obj", polygonSource);
  std::vector<std::array<float, 3>> vertexColors = mesh.getVertexColors("vertexColors.txt");
  std::vector<ngl::Vec3> interleavedData = mesh.interleavePosAndColor(polygonSource, vertexColors);

    
  // Generate indices from faces
  std::vector<GLshort> indices;
  for (Polygon_mesh::face_index f : faces(polygonSource))
  {
    for (vertex_descriptor v : vertices_around_face(polygonSource.halfedge(f), polygonSource))
    {
      indices.push_back(v);
    }

  }


  // Create and bind VAO
  m_vao = ngl::VAOFactory::createVAO(ngl::simpleIndexVAO, GL_TRIANGLES);
  m_vao->bind();


 m_vao->setData(ngl::SimpleIndexVAO::VertexData(
      interleavedData.size()*sizeof(ngl::Vec3),
      interleavedData[0].m_x,
      indices.size()*sizeof(indices[0]), &indices[0],
      GL_UNSIGNED_SHORT));

  // Set vertex attribute pointers
  m_vao->setVertexAttributePointer(0, 3, GL_FLOAT, sizeof(ngl::Vec3)*2, 0); 
  m_vao->setVertexAttributePointer(1, 3, GL_FLOAT, sizeof(ngl::Vec3)*2, 3); 

  // Specify the number of indices
  m_vao->setNumIndices(indices.size());

  // Unbind VAO
  m_vao->unbind();

          
}




void NGLScene::paintGL()
{
  // clear the screen and depth buffer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glViewport(0, 0, m_win.width, m_win.height);
  // Rotation based on the mouse position for our global transform
  auto rotX = ngl::Mat4::rotateX(m_win.spinXFace);
  auto rotY = ngl::Mat4::rotateY(m_win.spinYFace);

  // multiply the rotations
  m_mouseGlobalTX = rotY * rotX;
  // add the translations
  m_mouseGlobalTX.m_m[3][0] = m_modelPos.m_x;
  m_mouseGlobalTX.m_m[3][1] = m_modelPos.m_y;
  m_mouseGlobalTX.m_m[3][2] = m_modelPos.m_z;

  ngl::Mat4 MVP = m_project * m_view * m_mouseGlobalTX;
  ngl::ShaderLib::setUniform("MVP", MVP);

  m_vao->bind();
  m_vao->draw();
  m_vao->unbind();
}

//----------------------------------------------------------------------------------------------------------------------
void NGLScene::mouseMoveEvent(QMouseEvent *_event)
{
// note the method buttons() is the button state when event was called
// that is different from button() which is used to check which button was
// pressed when the mousePress/Release event is generated
#if QT_VERSION > QT_VERSION_CHECK(6, 0, 0)
  auto position = _event->position();
#else
  auto position = _event->pos();
#endif
  if (m_win.rotate && _event->buttons() == Qt::LeftButton)
  {
    int diffx = position.x() - m_win.origX;
    int diffy = position.y() - m_win.origY;
    m_win.spinXFace += static_cast<int>(0.5f * diffy);
    m_win.spinYFace += static_cast<int>(0.5f * diffx);
    m_win.origX = position.x();
    m_win.origY = position.y();
    update();
  }
  // right mouse translate code
  else if (m_win.translate && _event->buttons() == Qt::RightButton)
  {
    int diffX = static_cast<int>(position.x() - m_win.origXPos);
    int diffY = static_cast<int>(position.y() - m_win.origYPos);
    m_win.origXPos = position.x();
    m_win.origYPos = position.y();
    m_modelPos.m_x += INCREMENT * diffX;
    m_modelPos.m_y -= INCREMENT * diffY;
    update();
  }
}

//----------------------------------------------------------------------------------------------------------------------
void NGLScene::mousePressEvent(QMouseEvent *_event)
{
// that method is called when the mouse button is pressed in this case we
// store the value where the maouse was clicked (x,y) and set the Rotate flag to true
#if QT_VERSION > QT_VERSION_CHECK(6, 0, 0)
  auto position = _event->position();
#else
  auto position = _event->pos();
#endif
  if (_event->button() == Qt::LeftButton)
  {
    m_win.origX = position.x();
    m_win.origY = position.y();
    m_win.rotate = true;
  }
  // right mouse translate mode
  else if (_event->button() == Qt::RightButton)
  {
    m_win.origXPos = position.x();
    m_win.origYPos = position.y();
    m_win.translate = true;
  }
}

//----------------------------------------------------------------------------------------------------------------------
void NGLScene::mouseReleaseEvent(QMouseEvent *_event)
{
  // that event is called when the mouse button is released
  // we then set Rotate to false
  if (_event->button() == Qt::LeftButton)
  {
    m_win.rotate = false;
  }
  // right mouse translate mode
  if (_event->button() == Qt::RightButton)
  {
    m_win.translate = false;
  }
}

//----------------------------------------------------------------------------------------------------------------------
void NGLScene::wheelEvent(QWheelEvent *_event)
{

  // check the diff of the wheel position (0 means no change)
  if (_event->angleDelta().x() > 0)
  {
    m_modelPos.m_z += ZOOM;
  }
  else if (_event->angleDelta().x() < 0)
  {
    m_modelPos.m_z -= ZOOM;
  }
  update();
}

//----------------------------------------------------------------------------------------------------------------------

void NGLScene::keyPressEvent(QKeyEvent *_event)
{
  // this method is called every time the main window recives a key event.
  // we then switch on the key value and set the camera in the GLWindow
  switch (_event->key())
  {
  // escape key to quite
  case Qt::Key_Escape:
    QGuiApplication::exit(EXIT_SUCCESS);
    break;
  // turn on wirframe rendering
  case Qt::Key_W:
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    break;
  // turn off wire frame
  case Qt::Key_S:
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    break;
  // show full screen
  case Qt::Key_F:
    showFullScreen();
    break;
  // show windowed
  case Qt::Key_N:
    showNormal();
    break;
  default:
    break;
  }
  // finally update the GLWindow and re-draw
  // if (isExposed())
  update();
}
