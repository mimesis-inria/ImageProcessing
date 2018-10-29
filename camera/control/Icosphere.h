#ifndef SOFACV_CAM_CONTROL_ROTATEAROUNDENGINE_H
#define SOFACV_CAM_CONTROL_ROTATEAROUNDENGINE_H

#include "ImageProcessingPlugin.h"
#include "camera/common/CameraSettings.h"

#include <SofaCV/SofaCV.h>
#include <sofa/helper/OptionsGroup.h>

#define SAME_CLOCKNESS 1
#define DIFF_CLOCKNESS 0

namespace sofacv
{
namespace cam
{
namespace control
{

typedef struct fpoint_tag
{
   float x;
   float y;
   float z;
} fpoint;

struct Triangle
{
  int vertex[3];
};

using TriangleList=std::vector<Triangle>;
using VertexList=std::vector<sofa::defaulttype::Vector3>;

namespace icosahedron
{
const float X=0.525731112119133606;
const float Z=0.850650808352039932;
const float N=0.f;

static const VertexList vertices=
{
  {-X,N,Z}, {X,N,Z}, {-X,N,-Z}, {X,N,-Z},
  {N,Z,X}, {N,Z,-X}, {N,-Z,X}, {N,-Z,-X},
  {Z,X,N}, {-Z,X, N}, {Z,-X,N}, {-Z,-X, N}
};

static const TriangleList triangles=
{
  {0,4,1},{0,9,4},{9,5,4},{4,5,8},{4,8,1},
  {8,10,1},{8,3,10},{5,3,8},{5,2,3},{2,7,3},
  {7,10,3},{7,6,10},{7,11,6},{11,0,6},{0,1,6},
  {6,1,10},{9,0,11},{9,11,2},{9,2,5},{7,2,11}
};
}

using Lookup=std::map<std::pair<int, int>, int>;
using IndexedMesh=std::pair<VertexList, TriangleList>;


class SOFA_IMAGEPROCESSING_API Icosphere : public ImplicitDataEngine
{
  typedef sofa::core::objectmodel::SingleLink<
      Icosphere, cam::CameraSettings,
      sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK>
      CamSettings;
  typedef typename sofa::defaulttype::Matrix3 Matrix3;
  typedef typename sofa::defaulttype::Vector3 Vector3;
  typedef typename sofa::defaulttype::Quat Quat;

    IndexedMesh indexedmesh, indexedmeshclasses;

    std::vector<int> classes;

    std::vector<Vector3> centers;

    int index;
    std::ofstream file;

    int centerit;

 public:
  SOFA_CLASS(Icosphere, ImplicitDataEngine);

  Icosphere()
      : l_cam(initLink("cam", "camera to control")),
        d_center(initData(&d_center, "center",
                          "Point in world coordinates to rotate around")),
        d_theta(initData(&d_theta, "delta_theta",
                         "longitudinal angle to add at each step")),
        d_phi(initData(&d_phi, "delta_phi",
                       "polar (colatitude) angle to add at each step")),
        d_rho(initData(&d_rho, "delta_rho",
                       "distance to add to the sphere's radius at each step")),
        d_thetaInit(
            initData(&d_thetaInit, "theta", "Initial longitudinal angle")),
        d_phiInit(
            initData(&d_phiInit, "phi", "initial polar (colatitude) angle")),
        d_rhoInit(initData(&d_rhoInit, "rho", "initial sphere's radius"))
  {
  }

  virtual ~Icosphere() override{}
  void init() override
  {
    addInput(&d_center);
    addInput(&d_theta);
    addInput(&d_phi);
    addInput(&d_rho);

    if (!l_cam.get())
      msg_error(getName() + "::init()") << "Error: No camera link set. ";
    m_rho = d_rho.getValue();
    m_theta = d_theta.getValue();
    m_phi = d_phi.getValue();

    index = 0;

    int nsubdivisionsclasses = 1;
    int nsubdivisions = 5;

    indexedmesh = make_icosphere(nsubdivisions);
    indexedmeshclasses = make_icosphere(nsubdivisionsclasses);

    TriangleList triangles =  indexedmesh.second;
    VertexList vertices =  indexedmesh.first;

    VertexList verticesSemi;

    for (int k = 0; k <vertices.size(); k++)
    {
        if (vertices[k][0] < 0.01)
            verticesSemi.push_back(vertices[k]);
    }

    vertices = verticesSemi;
    indexedmesh.first = vertices;

    TriangleList trianglesclasses =  indexedmeshclasses.second;
    VertexList verticesclasses =  indexedmeshclasses.first;

    centers.resize(0);

    centerit = 0;


    //Pig1
    /*
    int nx = 3;
    int ny = 2;
    int nz = 3;
    double rangex = 150;
    double rangey = 150;
    double rangez = 150;*/

    //Patient 1

    int nx = 2;
    int ny = 2;
    int nz = 2;

    double rangex = 120;
    double rangey = 120;
    double rangez = 80;

    //Patient1

    /*double rangex = 120;
        double rangey = 120;
        double rangez = 100;*/

    /*int nx = 1;
        int ny = 1;
        int nz = 1;
        double rangex = 0;
        double rangey = 0;
        double rangez = 0;*/

    for (int kx = 0; kx < nx; kx++)
        for (int ky = 0; ky < ny; ky++)
            for (int kz = 0; kz < nz; kz++)
            {
                Vector3 cent;
                cent[0] = /*-0.5*rangex +*/ (double)kx*rangex/(nx-1);
                cent[1] =  -0.5*rangey + (double)ky*rangey/(ny-1);
                cent[2] =  -0.5*rangez + (double)kz*rangez/(nz-1);
                cent += d_center.getValue();
                centers.push_back(cent);
            }

    d_center.setValue(centers[0]);

    std::cout << " center0 " << d_center.getValue() << " inddex " << index << std::endl;

    classes.resize(vertices.size()*centers.size());

    rotate(0, 0, 0, true);

    std::string path = "classes.txt";
    std::string opath = "images/%06d_test.png";
    file.open(path.c_str(), std::ios_base::app );

    int im = 1;

    for (int kc = 0; kc < centers.size() ; kc++)
    {
    for (int k = 0; k <vertices.size(); k++)
    {
        char buf4[FILENAME_MAX];
        sprintf(buf4, opath.c_str(),im);
        std::string filename4(buf4);

        fpoint linep,vect;
        fpoint* pt_int;
        linep.x = vertices[k][0];
        linep.y = vertices[k][1];
        linep.z = vertices[k][2];

        vect.x = -vertices[k][0];
        vect.y = -vertices[k][1];
        vect.z = -vertices[k][2];

        int intersect;

        bool okintersect = false;

        std::cout << " vertices " << vertices[k] << " " << vertices.size() << "  " << trianglesclasses.size() <<  " vertices  " << verticesclasses.size() << std::endl;
        for (int l = 0; l < trianglesclasses.size(); l++)
        {
            fpoint p1, p2, p3;

            p1.x = verticesclasses[trianglesclasses[l].vertex[0]][0];
            p1.y = verticesclasses[trianglesclasses[l].vertex[0]][1];
            p1.z = verticesclasses[trianglesclasses[l].vertex[0]][2];

            p2.x = verticesclasses[trianglesclasses[l].vertex[1]][0];
            p2.y = verticesclasses[trianglesclasses[l].vertex[1]][1];
            p2.z = verticesclasses[trianglesclasses[l].vertex[1]][2];

            p3.x = verticesclasses[trianglesclasses[l].vertex[2]][0];
            p3.y = verticesclasses[trianglesclasses[l].vertex[2]][1];
            p3.z = verticesclasses[trianglesclasses[l].vertex[2]][2];

            pt_int = new fpoint;

            intersect = check_intersect_tri(p1, p2, p3, linep, vect, pt_int);

            if (intersect == 1)
            {
                classes[k]=l + trianglesclasses.size()*kc;
                okintersect = true;
            }
        }

        if (okintersect==true)
        {
        file << filename4;
        file << " ";
        file << classes[k];
        file << "\n";
        }
        im++;
    }
    }

       file.close();

  }

  int vertex_for_edge(Lookup& lookup,
    VertexList& vertices, int first, int second)
  {
    Lookup::key_type key(first, second);
    if (key.first>key.second)
      std::swap(key.first, key.second);

    auto inserted=lookup.insert({key, vertices.size()});
    if (inserted.second)
    {
      auto& edge0=vertices[first];
      auto& edge1=vertices[second];
      auto point=(edge0+edge1);
      point.normalize();
      vertices.push_back(point);
    }

    return inserted.first->second;
  }

  TriangleList subdivide(VertexList& vertices,
    TriangleList triangles)
  {
    Lookup lookup;
    TriangleList result;

    for (auto&& each:triangles)
    {
      std::array<int, 3> mid;
      for (int edge=0; edge<3; ++edge)
      {
        mid[edge]=vertex_for_edge(lookup, vertices,
          each.vertex[edge], each.vertex[(edge+1)%3]);
      }

      result.push_back({each.vertex[0], mid[0], mid[2]});
      result.push_back({each.vertex[1], mid[1], mid[0]});
      result.push_back({each.vertex[2], mid[2], mid[1]});
      result.push_back({mid[0], mid[1], mid[2]});
    }

    return result;
  }

  IndexedMesh make_icosphere(int subdivisions)
  {
    VertexList vertices=icosahedron::vertices;
    TriangleList triangles=icosahedron::triangles;

    for (int i=0; i<subdivisions; ++i)
    {
      triangles=subdivide(vertices, triangles);
    }

    return{vertices, triangles};
  }

  int check_same_clock_dir(fpoint pt1, fpoint pt2, fpoint pt3, fpoint norm)
  {
     float testi, testj, testk;
     float dotprod;
     // normal of trinagle
     testi = (((pt2.y - pt1.y)*(pt3.z - pt1.z)) - ((pt3.y - pt1.y)*(pt2.z - pt1.z)));
     testj = (((pt2.z - pt1.z)*(pt3.x - pt1.x)) - ((pt3.z - pt1.z)*(pt2.x - pt1.x)));
     testk = (((pt2.x - pt1.x)*(pt3.y - pt1.y)) - ((pt3.x - pt1.x)*(pt2.y - pt1.y)));

     // Dot product with triangle normal
     dotprod = testi*norm.x + testj*norm.y + testk*norm.z;

     //answer
     if(dotprod < 0){ return DIFF_CLOCKNESS;}
     else {return SAME_CLOCKNESS;}
  }

  int check_intersect_tri(fpoint pt1, fpoint pt2, fpoint pt3, fpoint linept, fpoint vect,
                          fpoint* pt_int)
  {
     float V1x, V1y, V1z;
     float V2x, V2y, V2z;
     fpoint norm;
     float dotprod;
     float t;

     // vector form triangle pt1 to pt2
     V1x = pt2.x - pt1.x;
     V1y = pt2.y - pt1.y;
     V1z = pt2.z - pt1.z;

     // vector form triangle pt2 to pt3
     V2x = pt3.x - pt2.x;
     V2y = pt3.y - pt2.y;
     V2z = pt3.z - pt2.z;

     // vector normal of triangle
     norm.x = V1y*V2z-V1z*V2y;
     norm.y = V1z*V2x-V1x*V2z;
     norm.z = V1x*V2y-V1y*V2x;

     // dot product of normal and line's vector if zero line is parallel to triangle
     dotprod = norm.x*vect.x + norm.y*vect.y + norm.z*vect.z;

     //if(dotprod < 0)
     {
        //Find point of intersect to triangle plane.
        //find t to intersect point
        t = -(norm.x*(linept.x-pt1.x)+norm.y*(linept.y-pt1.y)+norm.z*(linept.z-pt1.z))/(norm.x*vect.x+norm.y*vect.y+norm.z*vect.z);

        // if ds is neg line started past triangle so can't hit triangle.
        if(t < -0.001 || t >1) {return 0;}

        pt_int->x = linept.x + vect.x*t;
        pt_int->y = linept.y + vect.y*t;
        pt_int->z = linept.z + vect.z*t;

        //std::cout << " vectx " << vect.x*t << " vecty " << vect.y*t << " vectz " << vect.z*t << std::endl;

        if(check_same_clock_dir(pt1, pt2, *pt_int, norm) == SAME_CLOCKNESS)
        {
           if(check_same_clock_dir(pt2, pt3, *pt_int, norm) == SAME_CLOCKNESS)
           {
              if(check_same_clock_dir(pt3, pt1, *pt_int, norm) == SAME_CLOCKNESS)
              {
                  std::cout << " t " << t << " norm " << norm.x*vect.x+norm.y*vect.y+norm.z*vect.z << " pt1 " << pt1.x << " " << pt1.y << " " << pt1.z << " pt2 " << " " << pt2.x << " " << pt2.y << " " << pt2.z << " pt3 " << pt3.x << " " << pt3.y << " " << pt3.z <<std::endl;

                 // answer in pt_int is insde triangle
                 return 1;
              }
           }
        }
     }
     return 0;
  }

  void rotate(double rho, double theta, double phi, bool init = false)
  {

    Vector3 p = Vector3(0, 0, d_rhoInit.getValue());
    Vector3 sphCoord;

    TriangleList triangles =  indexedmesh.second;
    VertexList vertices =  indexedmesh.first;


    /*for (int k = 0; k <vertices.size(); k++)
    {
        std::cout << " vertices " << vertices[k] << std::endl;
    }

    // convert to spherical coordinates and add rho, theta & phi
    // rho
    sphCoord.x() =
        std::sqrt((p.x() * p.x()) + (p.y() * p.y()) + (p.z() * p.z()));
    // theta
    sphCoord.y() = d_thetaInit.getValue() + std::acos(p.z() / sphCoord.x());
    // phi
    sphCoord.z() = d_phiInit.getValue() + std::atan2(p.y(), p.x());

    if (init)
    {
    }
    else
    {
      sphCoord.x() += m_rho;
      sphCoord.x() = std::abs(sphCoord.x());
      sphCoord.y() += m_theta;
      sphCoord.z() += m_phi;
      m_rho += rho;
      m_theta += theta;
      m_phi += phi;
    }*/
    // convert back to cartesian coordinates
    /*p.x() = sphCoord.x() * std::sin(sphCoord.y()) * std::cos(sphCoord.z());
    p.y() = sphCoord.x() * std::sin(sphCoord.y()) * std::sin(sphCoord.z());
    p.z() = sphCoord.x() * std::cos(sphCoord.y());*/

    p.x() = vertices[index][0]*d_rhoInit.getValue();
    p.y() = vertices[index][1]*d_rhoInit.getValue();
    p.z() = vertices[index][2]*d_rhoInit.getValue();

    sphCoord.x() =
    std::sqrt((p.x() * p.x()) + (p.y() * p.y()) + (p.z() * p.z()));
    // theta
    sphCoord.y() = /*d_thetaInit.getValue() +*/ std::acos(p.z() / sphCoord.x());
    // phi
    sphCoord.z() = /*d_phiInit.getValue() +*/ std::atan2(p.y(), p.x());

    std::cout << "vertex " << p.x() << " " << p.y() << " " << p.z() <<std::endl;

    index++;

    Quat q1 = Quat(Vector3(1.0, 0.0, 0.0), M_PI);
    Quat q2 = Quat::fromEuler(0, -sphCoord.y() + M_PI, 0);
    Quat q3 = Quat::fromEuler(0, 0, -sphCoord.z());
    Quat q4 = Quat(Vector3(0.0, 0.0, 1.0), -M_PI / 2);
    Quat q5 = Quat(Vector3(0.0, 1.0, 0.0), M_PI / 2);
    Quat q6 = Quat(Vector3(1.0, 0.0, 0.0), -M_PI / 2);
    Quat q7 = Quat(Vector3(0.0, 0.0, 1.0), -M_PI / 2);


    Matrix3 R1, R2, R3, R4, R5, R6, R7;
    q1.toMatrix(R1);
    q2.toMatrix(R2);
    q3.toMatrix(R3);
    q4.toMatrix(R4);
    q5.toMatrix(R5);
    q6.toMatrix(R6);
    q7.toMatrix(R7);

    p = Vector3(-p.z(), -p.x(), p.y());

    std::cout << " p " << p << std::endl;

    if (index == vertices.size())
        {
        centerit++;
        d_center.setValue(centers[centerit]);
        index = 0;
        }
    std::cout << " center " << d_center.getValue() << " inddex " << index << std::endl;

    p += d_center.getValue();

    std::cout << " center " << d_center.getValue() << std::endl;

    Matrix3 R = R7 * R4 * R2 * R3 * R5 * R6;

    l_cam->setPosition(p, false);
    l_cam->setRotationMatrix(R, false);
    l_cam->buildFromKRT();
  }

  void Update() override
  {
    if (m_dataTracker.isDirty(d_center))
      centerChanged();
    if (m_dataTracker.isDirty(d_theta))
      thetaChanged();
    if (m_dataTracker.isDirty(d_phi))
      phiChanged();
    if (m_dataTracker.isDirty(d_rho))
      rhoChanged();
    rotate(d_rho.getValue(), d_theta.getValue(), d_phi.getValue());
  }

  virtual void handleEvent(sofa::core::objectmodel::Event* e) override
  {
    if (sofa::simulation::AnimateBeginEvent::checkEventType(e))
    {
      update();
    }
  }

  CamSettings l_cam;
  sofa::Data<Vector3> d_center;
  sofa::Data<double> d_theta;
  sofa::Data<double> d_phi;
  sofa::Data<double> d_rho;

  Vector3 m_pole;
  sofa::Data<double> d_thetaInit;
  sofa::Data<double> d_phiInit;
  sofa::Data<double> d_rhoInit;

  double m_theta;
  double m_phi;
  double m_rho;

 private:
  void centerChanged() {}
  void thetaChanged()
  {
    //      m_theta = d_theta.getValue();
  }
  void phiChanged()
  {
    //      m_phi = d_phi.getValue();
  }
  void rhoChanged()
  {
    //      m_rho = d_rho.getValue();
  }

  void thetaInitChanged()
  {
    rotate(0, 0, 0, true);
  }
  void phiInitChanged()
  {
    rotate(0, 0, 0, true);
  }
  void rhoInitChanged()
  {
    rotate(0, 0, 0, true);
  }
};

SOFA_DECL_CLASS(Icosphere)

int IcosphereClass =
    sofa::core::RegisterObject(
        "Component to rotate a camera following any trajectory on a sphere, "
        "around a point. Init sets the correct camera orientation if necessary")
        .add<Icosphere>();

}  // namespace control
}  // namespace cam
}  // namespace sofacv
#endif  // SOFACV_CAM_CONTROL_ROTATEAROUNDENGINE_H
