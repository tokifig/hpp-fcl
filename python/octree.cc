
#include "fcl.hh"

#include <hpp/fcl/fwd.hh>
#include <hpp/fcl/octree.h>

#ifdef HPP_FCL_HAS_DOXYGEN_AUTODOC
#include "doxygen_autodoc/functions.h"
#endif

void exposeOctree() {
  using namespace hpp::fcl;
  namespace bp = boost::python;
  namespace dv = doxygen::visitor;

  bp::class_<std::pair<Eigen::Matrix<FCL_REAL, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<int32_t, Eigen::Dynamic, Eigen::Dynamic>>>("OcTreeMesh")
    .def("vertices", +[](const std::pair<Eigen::Matrix<FCL_REAL, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<int32_t, Eigen::Dynamic, Eigen::Dynamic>>& mesh) {
        return mesh.first;
    })
    .def("faces", +[](const std::pair<Eigen::Matrix<FCL_REAL, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<int32_t, Eigen::Dynamic, Eigen::Dynamic>>& mesh) {
        return mesh.second;
    });

  bp::class_<OcTree, bp::bases<CollisionGeometry>, shared_ptr<OcTree> >(
      "OcTree", doxygen::class_doc<OcTree>(), bp::no_init)
      .def(dv::init<OcTree, FCL_REAL>())
      .def(dv::member_func("computeMesh", +[](const OcTree& octree) {
        const auto vertices_faces = octree.computeMesh();
        const auto& vertices = vertices_faces.first;
        const auto& faces = vertices_faces.second;

        Eigen::Matrix<FCL_REAL, Eigen::Dynamic, Eigen::Dynamic> vs = Eigen::Map<const Eigen::Matrix<FCL_REAL, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
           reinterpret_cast<const FCL_REAL *>(vertices.data()), vertices.size(), 3
        );
        Eigen::Matrix<int32_t, Eigen::Dynamic, Eigen::Dynamic> fs = Eigen::Map<const Eigen::Matrix<size_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
           reinterpret_cast<const size_t *>(faces.data()), faces.size(), 4
        ).cast<int32_t>();

        return std::make_pair(vs, fs);
      }))
      .def(dv::member_func("exportAsObjFile", &OcTree::exportAsObjFile))
      .def(dv::member_func("toBoxes", +[](const OcTree& octree) {
         const auto boxes = octree.toBoxes();
         Eigen::Matrix<FCL_REAL, Eigen::Dynamic, Eigen::Dynamic> mat;
         mat = Eigen::Map<const Eigen::Matrix<FCL_REAL, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
           reinterpret_cast<const FCL_REAL *>(boxes.data()), boxes.size(), 6
         );
         return mat;
       }))
      .def(dv::member_func("getTreeDepth", &OcTree::getTreeDepth))
      .def(dv::member_func("getOccupancyThres", &OcTree::getOccupancyThres))
      .def(dv::member_func("getFreeThres", &OcTree::getFreeThres))
      .def(dv::member_func("getDefaultOccupancy", &OcTree::getDefaultOccupancy))
      .def(dv::member_func("setCellDefaultOccupancy",
                           &OcTree::setCellDefaultOccupancy))
      .def(dv::member_func("setOccupancyThres", &OcTree::setOccupancyThres))
      .def(dv::member_func("setFreeThres", &OcTree::setFreeThres))
      .def(dv::member_func("getRootBV", &OcTree::getRootBV));

  doxygen::def("makeOctree", &makeOctree);
}
