#define CGAL_EIGEN3_ENABLED
#define CGAL_PMP_REMESHING_VERBOSE

#include <iostream>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <vector>
#include <emscripten.h>
#include <emscripten/bind.h>
#include <CGAL/subdivision_method_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_3.h>
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <boost/foreach.hpp>
#include <CGAL/Surface_mesh_parameterization/IO/File_off.h>
#include <CGAL/Surface_mesh_parameterization/parameterize.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>
#include <igl/PI.h>
#include <CGAL/Dimension.h>
#include <iostream>
#include <fstream>
#include <CGAL/Surface_mesh_parameterization/Circular_border_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Discrete_authalic_parameterizer_3.h>
#include <CGAL/Surface_mesh_parameterization/Error_code.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/detect_features.h>
#include <CGAL/Polygon_mesh_processing/surface_Delaunay_remeshing.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Mesh_constant_domain_field_3.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/mesh_segmentation.h>
#include <CGAL/property_map.h>
#include <CGAL/Polyhedron_3.h>

#include <CGAL/Mesh_triangulation_3.h>
#include <CGAL/Mesh_complex_3_in_triangulation_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/boost/graph/helpers.h>
#include <CGAL/Polyhedral_mesh_domain_3.h>
#include <CGAL/make_mesh_3.h>
#include <CGAL/refine_mesh_3.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/boost/graph/convert_nef_polyhedron_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/angle_and_area_smoothing.h>
#include <CGAL/Polygon_mesh_processing/repair_degeneracies.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Polyhedral_mesh_domain_with_features_3<K> Mesh_domain;
// Polyhedron type
typedef CGAL::Mesh_polyhedron_3<K>::type Polyhedron;
// Triangulation
typedef CGAL::Mesh_triangulation_3<Mesh_domain>::type Tr;
typedef CGAL::Mesh_complex_3_in_triangulation_3<
  Tr,Mesh_domain::Corner_index,Mesh_domain::Curve_index> C3t3;
// Criteria
typedef CGAL::Mesh_criteria_3<Tr> Mesh_criteria;

typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Surface_mesh<Kernel::Point_3> SurfaceMesh;
typedef boost::graph_traits<SurfaceMesh>::halfedge_descriptor        halfedge_descriptor;
typedef boost::graph_traits<SurfaceMesh>::edge_descriptor            edge_descriptor;

namespace PMP = CGAL::Polygon_mesh_processing;

struct halfedge2edge
{
  halfedge2edge(const SurfaceMesh& m, std::vector<edge_descriptor>& edges)
    : m_mesh(m), m_edges(edges)
  {}
  void operator()(const halfedge_descriptor& h) const
  {
    m_edges.push_back(edge(h, m_mesh));
  }
  const SurfaceMesh& m_mesh;
  std::vector<edge_descriptor>& m_edges;
};

class PolyMesh {
    private:
        std::vector<double> vertices;
        std::vector<int> faces;
        SurfaceMesh mesh;
        // SurfaceMesh triangulatedMesh;
    public:
        Kernel::Point_3 sdf_p = Kernel::Point_3(0, 4, 0);
        PolyMesh() {
            std::cout << "created mesh" << std::endl;
        }
        //CGAL::SM_Vertex_index
        int addVertex(double x, double y, double z) {
            auto v0 = mesh.add_vertex(Kernel::Point_3(x, y, z));
            return v0.idx();
        }
        // int addFaceNative(int x, int y, int z) {
        //     std::vector<SurfaceMesh::vertex_index> faceVerts;
        //     faceVerts.push_back(x);
        //     faceVerts.push_back(y);
        //     faceVerts.push_back(z);
        //     auto fc = mesh.add_face(faceVerts);
        //     return fc.idx();
        // }
        int addFace(emscripten::val vertices){
            std::vector<SurfaceMesh::vertex_index> faceVerts;
            for (int i = 0; i < vertices["length"].as<int>(); i++) {
                faceVerts.push_back(mesh.vertices().begin()[vertices[i].as<int>()]);
            }
            auto fc = mesh.add_face(faceVerts);
            return fc.idx();
        }
        void triangulate(SurfaceMesh & targetMesh){
            CGAL::Polygon_mesh_processing::triangulate_faces(targetMesh);
        }
        void catmull_smooth(){
            CGAL::Subdivision_method_3::CatmullClark_subdivision(mesh, 1);
        }
        void loop_smooth(){
            CGAL::Subdivision_method_3::Loop_subdivision(mesh, 1);
        }
        void dooSabin_smooth(){
            CGAL::Subdivision_method_3::DooSabin_subdivision(mesh, 1);
        }
        void sqrt_smooth(){
            CGAL::Subdivision_method_3::Sqrt3_subdivision(mesh, 1);
        }
        void decimate(double stop_ratio){
            namespace SMS = CGAL::Surface_mesh_simplification;
            std::cout << "decimate stop ratio" << stop_ratio << std::endl;
            SMS::Count_ratio_stop_predicate<SurfaceMesh> stop(stop_ratio);
            int r = SMS::edge_collapse(mesh, stop);
            std::cout << "\nFinished!\n" << r << " edges removed.\n" << mesh.number_of_edges() << " final edges.\n";
        }
        
        static void defineSDF(emscripten::val call_back){
            // auto m = call_back().as<int>();
            //call_back["length"].as<int>();
            // std::cout<<"ping callback: "<<m<<std::endl;
            // call_back();

        }

        void parametizeMesh(){
            typedef Kernel::Point_2 Point_2;
            typedef Kernel::Point_3 Point_3;

            typedef boost::graph_traits<SurfaceMesh>::vertex_descriptor     vertex_descriptor;
            typedef boost::graph_traits<SurfaceMesh>::halfedge_descriptor   halfedge_descriptor;
            typedef boost::graph_traits<SurfaceMesh>::face_descriptor       face_descriptor;

            namespace SMP = CGAL::Surface_mesh_parameterization;
            halfedge_descriptor bhd = CGAL::Polygon_mesh_processing::longest_border(mesh).first;
            // The UV property map that holds the parameterized values
            typedef SurfaceMesh::Property_map<vertex_descriptor, Point_2>  UV_pmap;
            std::cout << "before parametization" << std::endl;
            UV_pmap uv_map = mesh.add_property_map<vertex_descriptor, Point_2>("h:uv").first;
            
            typedef SMP::Circular_border_arc_length_parameterizer_3<SurfaceMesh>  Border_parameterizer;
            typedef SMP::Discrete_authalic_parameterizer_3<SurfaceMesh, Border_parameterizer> Parameterizer;

            SMP::parameterize(mesh, Parameterizer(), bhd, uv_map);
            std::cout << "after parametization" << std::endl;
            for(auto uv : uv_map){
                std::cout<<uv.x()<<" "<<uv.y()<<std::endl;
            }
        }

        emscripten::val getIndices(){
            std::vector<int> indices;
            SurfaceMesh triangulatedMesh = mesh;
            if(!this->isTriangulated(mesh)){
                std::cout<<"non-triangular mesh, triangulating..."<<std::endl;
                this->triangulate(triangulatedMesh); //triangulate first
            }
            
             for(auto faceIt = triangulatedMesh.faces_begin(); faceIt != triangulatedMesh.faces_end(); ++faceIt){
                //get the vertices of the face
                auto halfedgeIt = triangulatedMesh.halfedge(*faceIt);
                auto halfedgeIt2 = triangulatedMesh.next(halfedgeIt);
                auto halfedgeIt3 = triangulatedMesh.next(halfedgeIt2);
                auto vertexIt = triangulatedMesh.target(halfedgeIt);
                auto vertexIt2 = triangulatedMesh.target(halfedgeIt2);
                auto vertexIt3 = triangulatedMesh.target(halfedgeIt3);
                indices.push_back(vertexIt.idx());
                indices.push_back(vertexIt2.idx());
                indices.push_back(vertexIt3.idx());
            }
            triangulatedMesh.clear();
            triangulatedMesh.collect_garbage();
            return emscripten::val(emscripten::typed_memory_view(indices.size(), indices.data()));
        }

        bool isTriangulated(const SurfaceMesh& mesh) {
            for (const auto& f : mesh.faces()) {
                if (mesh.degree(f) != 3) {
                    return false;  // Face does not have 3 vertices, not triangulated
                }
            }
            return true;
        }

        void remesh_isotropic() {
            // Polyhedron poly;
            // CGAL::copy_face_graph(mesh, poly);
            // std::vector<Polyhedron::Facet_handle>  new_facets;
            // std::vector<Polyhedron::Vertex_handle> new_vertices;

            // auto fo = new_facets[0];

            // PMP::refine(mesh, CGAL::faces(mesh),
            //   std::back_inserter(new_facets),
            //   std::back_inserter(new_vertices),
            //   CGAL::parameters::density_control_factor(0.5));

            // mesh.clear();
            // SurfaceMesh m;
            // for (const auto v : new_vertices) {
            //     m.add_vertex(v.node->point);
            // }
            // for (const auto f : new_facets) {
            //     f.node->halfedge
            // }

            // mesh.collect_garbage();
            // CGAL::copy_face_graph(poly, mesh);
            // poly.clear();

            double target_edge_length = 0.5;
            unsigned int nb_iter = 1;
            std::vector<edge_descriptor> border;
            PMP::border_halfedges(mesh.faces(), mesh, boost::make_function_output_iterator(halfedge2edge(mesh, border)));
            PMP::split_long_edges(border, target_edge_length, mesh);
            // PMP::remove_almost_degenerate_faces(CGAL::faces(mesh), mesh);
            PMP::orient_to_bound_a_volume(mesh);
            // PMP::angle_and_area_smoothing(mesh.faces(), mesh, CGAL::parameters::use_area_smoothing(false));
            // PMP::isotropic_remeshing(mesh.faces(), target_edge_length, mesh,
            //         CGAL::parameters::number_of_iterations(nb_iter)
            //             .do_flip(false)
            //             .do_project(true)
            //             .do_split(true)
            //             .protect_constraints(true)); //i.e. protect border, here
        }

        PolyMesh remesh_delaunay() {
            // using EIFMap = boost::property_map<SurfaceMesh, CGAL::edge_is_feature_t>::type;
            // EIFMap eif = get(CGAL::edge_is_feature, mesh);
            // PMP::detect_sharp_edges(mesh, 45, eif);
            // std::cout << "sharp edges " << eif. << std::endl;

            SurfaceMesh outmesh = PMP::surface_Delaunay_remeshing(mesh,
                                  CGAL::parameters::mesh_edge_size(0.75)
                                    .mesh_facet_size(1)
                                    .mesh_facet_topology(CGAL::FACET_VERTICES_ON_SURFACE)
            );
            // std::vector<double> vertices;
            PolyMesh polyMesh = PolyMesh();
            polyMesh.mesh = outmesh;
            // for(auto vertexIt = outmesh.vertices_begin(); vertexIt != outmesh.vertices_end(); ++vertexIt){
            //     auto point = outmesh.point(*vertexIt);
            //     polyMesh.addVertex(point.x(), point.y(), point.z());
            // }
            // for(auto facesIt = outmesh.faces_begin(); vertexIt != outmesh.faces_end(); ++facesIt){
            //     auto fa = outmesh.face(*facesIt);
            //     polyMesh.addVertex(point.x(), point.y(), point.z());
            // }
            return polyMesh;
        }

        PolyMesh polyhedral_mesh_generation() {
            PolyMesh polyMesh = PolyMesh();
            Polyhedron polyhedron;
            CGAL::copy_face_graph(mesh, polyhedron);
            Mesh_domain domain(polyhedron);

            if (!CGAL::is_triangle_mesh(polyhedron)){
                std::cerr << "Input geometry is not triangulated." << std::endl;
                return polyMesh;
            } else {
                // Mesh criteria (no cell_size set)
                Mesh_criteria criteria(CGAL::parameters::facet_angle(25)
                    .facet_size(0.5)
                    // .facet_distance(0.008)
                    // .cell_radius_edge_ratio(3)
                );
                // Mesh generation
                C3t3 c3t3 = CGAL::make_mesh_3<C3t3>(domain, criteria, CGAL::parameters::no_perturb().no_exude());

                // Set tetrahedron size (keep cell_radius_edge_ratio), ignore facets
                // Mesh_criteria new_criteria(CGAL::parameters::cell_radius_edge_ratio(3).cell_size(0.03));

                // CGAL::refine_mesh_3(c3t3, domain, new_criteria, CGAL::parameters::manifold());

                CGAL::facets_in_complex_3_to_triangle_mesh(c3t3, polyMesh.mesh);

                // CGAL::convert_nef_polyhedron_to_polygon_mesh(domain, polyMesh.mesh);
                // CGAL::copy_face_graph(c3t3, polyMesh.mesh);
                return polyMesh;
            }

        }

        emscripten::val getVertices(){
            std::vector<double> vertices;
            for(auto vertexIt = mesh.vertices_begin(); vertexIt != mesh.vertices_end(); ++vertexIt){
                auto point = mesh.point(*vertexIt);
                vertices.push_back(point.x());
                vertices.push_back(point.y());
                vertices.push_back(point.z());
            }
            return emscripten::val(emscripten::typed_memory_view(vertices.size(), vertices.data()));
        }

        emscripten::val segment(int n_clusters, double sm_lambda){
            typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
            
            typedef boost::graph_traits<SurfaceMesh>::vertex_descriptor vertex_descriptor;
            typedef boost::graph_traits<SurfaceMesh>::face_descriptor face_descriptor;

            // create a property-map
            typedef SurfaceMesh::Property_map<face_descriptor,double> Facet_double_map;
            Facet_double_map sdf_property_map;
            sdf_property_map = mesh.add_property_map<face_descriptor,double>("f:sdf").first;
            // compute SDF values
            // We can't use default parameters for number of rays, and cone angle
            // and the postprocessing
            std::cout<<"before sdf values"<<std::endl;
            CGAL::sdf_values(mesh, sdf_property_map, 2.0 / 3.0 * CGAL_PI, 25, true);
            // create a property-map for segment-ids
            typedef SurfaceMesh::Property_map<face_descriptor, std::size_t> Facet_int_map;
            Facet_int_map segment_property_map = mesh.add_property_map<face_descriptor,std::size_t>("f:sid").first;;

            const std::size_t number_of_clusters = n_clusters;       // use 4 clusters in soft clustering
            const double smoothing_lambda = sm_lambda;  // importance of surface features, suggested to be in-between [0,1]

            std::size_t number_of_segments;
            if(n_clusters == 0 && sm_lambda == 0)
            {
                std::cout<<"using default parameters"<<std::endl;
                number_of_segments = CGAL::segmentation_from_sdf_values(mesh, sdf_property_map, segment_property_map);
            } else
            {
                std::cout<<"using custom parameters"<<std::endl;
                number_of_segments = CGAL::segmentation_from_sdf_values(mesh, sdf_property_map, segment_property_map, number_of_clusters, smoothing_lambda);
            }
            std::cout<<"sdf values"<<std::endl;
            std::cout << "Number of segments: " << number_of_segments << std::endl;
            //get face descriptor os surfacemesh
            std::vector<int> segments;
            for(auto faceIt = mesh.faces_begin(); faceIt != mesh.faces_end(); ++faceIt){
                std::cout << segment_property_map[*faceIt] << " ";
                segments.push_back(segment_property_map[*faceIt]);
            }
            std::cout << std::endl;
            return emscripten::val(emscripten::typed_memory_view(segments.size(), segments.data()));
        }
};

EMSCRIPTEN_BINDINGS(my_module) {
  emscripten::class_<PolyMesh>("PolyMesh")
    .constructor<>()
    .function("addVertex", &PolyMesh::addVertex, emscripten::allow_raw_pointers())
    .function("addFace", &PolyMesh::addFace, emscripten::allow_raw_pointers())
    .function("getIndices", &PolyMesh::getIndices, emscripten::allow_raw_pointers())
    .function("getVertices", &PolyMesh::getVertices, emscripten::allow_raw_pointers())
    .function("parametizeMesh", &PolyMesh::parametizeMesh, emscripten::allow_raw_pointers())
    .function("catmull_smooth", &PolyMesh::catmull_smooth, emscripten::allow_raw_pointers())
    .function("loop_smooth", &PolyMesh::loop_smooth, emscripten::allow_raw_pointers())
    .function("sqrt_smooth", &PolyMesh::sqrt_smooth, emscripten::allow_raw_pointers())
    .function("dooSabin_smooth", &PolyMesh::dooSabin_smooth, emscripten::allow_raw_pointers())
    .function("segment", &PolyMesh::segment, emscripten::allow_raw_pointers())
    .function("decimate", &PolyMesh::decimate, emscripten::allow_raw_pointers())
    .function("polyhedral_mesh_generation", &PolyMesh::polyhedral_mesh_generation, emscripten::allow_raw_pointers())
    .function("remesh_delaunay", &PolyMesh::remesh_delaunay, emscripten::allow_raw_pointers())
    .function("remesh_isotropic", &PolyMesh::remesh_isotropic, emscripten::allow_raw_pointers());
}