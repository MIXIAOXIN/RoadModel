#include <string>
#include <iostream>
#include <CGAL/Spherical_kernel_3.h>
#include <CGAL/Point_3.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Triangulation_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/draw_triangulation_3.h>
#include <CGAL/draw_triangulation_2.h>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <CGAL/boost/graph/copy_face_graph.h>
#include <CGAL/boost/graph/helpers.h>



//#include <Vec3.hpp>
//#include <threeCGAL.h>
//#include <gdal_priv.h>
#include <CGAL/Triangulation_vertex_base_with_id_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_face_base_2.h>
#include <CGAL/boost/graph/graph_traits_Delaunay_triangulation_2.h>
#include <CGAL/boost/graph/copy_face_graph.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>


#include <CGAL/Polygon_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>


#include <CGAL/compute_average_spacing.h>
#include <CGAL/Polygon_mesh_processing/locate.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <boost/graph/adjacency_list.hpp>
#include <CGAL/boost/graph/split_graph_into_polylines.h>
#include <CGAL/IO/WKT.h>
#include <CGAL/IO/OBJ.h>
#include <CGAL/Polyline_simplification_2/simplify.h>
#include <CGAL/Polyline_simplification_2/Squared_distance_cost.h>
#include <CGAL/Classification.h>
#include <CGAL/Random.h>
#include <fstream>
#include <queue>
#include <vector>


#include "shpline_io_shplib.h"

struct FaceInfo2
{
    FaceInfo2(){}
    int nesting_level;
    bool in_domain()
    {
        return nesting_level % 2 == 1;
    }
};

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Projection_traits = CGAL::Projection_traits_xy_3<Kernel>;
using GT = CGAL::Projection_traits_3<Kernel>;
using Point_2 = Kernel::Point_2;
using Point_3 = Kernel::Point_3;
using Segment_3 = Kernel::Segment_3;
using Polygon_2 = CGAL::Polygon_2<Kernel>;
using Polygon_3 = CGAL::Polygon_2<Projection_traits>;
using Itag = CGAL::Exact_intersections_tag;
using Ptag = CGAL::Exact_predicates_tag;
namespace PS = CGAL::Polyline_simplification_2;
using CDT_vertex_base = PS::Vertex_base_2<Projection_traits>;
using DT_vertex_id = CGAL::Triangulation_vertex_base_with_id_2<Projection_traits>;
using DT_face_info = CGAL::Triangulation_face_base_with_info_2<FaceInfo2, Kernel>;
using CDT_face_base = CGAL::Constrained_triangulation_face_base_2<Projection_traits , DT_face_info>;
using CDT_TDS = CGAL::Triangulation_data_structure_2<DT_vertex_id, CDT_face_base>;
using CDT = CGAL::Constrained_Delaunay_triangulation_2<Projection_traits , CDT_TDS, Ptag>;
using CTP = CGAL::Constrained_triangulation_plus_2<CDT>;
using Vertex_handlle = CDT::Vertex_handle;



// Triangulated Irregular Network
using TIN = CGAL::Delaunay_triangulation_2<Projection_traits>;

#ifdef CGAL_LINKED_WITH_TBB
using Concurrency_tag = CGAL::Parallel_tag;
#else
using Concurrency_tag = CGAL::Sequential_tag;
#endif

void CDT_Vertex_transfer(CDT_vertex_base vt_in, Point_3& pt){
    pt = vt_in.point();
}

std::vector<std::vector<Point_3>> convert_point_struct_lines(std::vector<std::vector<Eigen::Vector3d>> input_pts)
{
    std::vector<std::vector<Point_3>> output_pts;
    for (int i = 0; i < input_pts.size(); ++i) {
        std::vector<Point_3> tmp_pts;
        for (int j = 0; j < input_pts[i].size(); ++j) {
            Point_3 pt(input_pts[i][j].x(), input_pts[i][j].y(), input_pts[i][j].z());
            tmp_pts.push_back(pt);
        }
        output_pts.push_back(tmp_pts);
    }
    return output_pts;
}

std::vector<Polygon_3> convert_point_struct_polygons(std::vector<std::vector<Eigen::Vector3d>> input_pts)
{
    std::vector<Polygon_3> output_polys;
    for (int i = 0; i < input_pts.size(); ++i) {
        Polygon_3 tmp_plygon;
        for (int j = 0; j < input_pts[i].size(); ++j) {
            Point_3 pt(input_pts[i][j].x(), input_pts[i][j].y(), input_pts[i][j].z());
            tmp_plygon.push_back(pt);
        }
        output_polys.push_back(tmp_plygon);
    }
    return output_polys;
}

CGAL::Point_set_3<Point_3> convert_point_struct2(std::vector<std::vector<Eigen::Vector3d>> input_pts)
{
    CGAL::Point_set_3<Point_3> output_pts;
    for (int i = 0; i < input_pts.size(); ++i) {
        for (int j = 0; j < input_pts[i].size(); ++j) {
            Point_3 pt(input_pts[i][j].x(), input_pts[i][j].y(), input_pts[i][j].z());
            output_pts.insert(pt);
        }
    }

    return output_pts;
}

void write_triangle_poly_file(const TIN& t, std::ostream &f) {
    typedef typename TIN::Vertex_handle Vertex_handle;
    typedef typename TIN::Finite_vertices_iterator
            Finite_vertices_iterator;
    typedef typename TIN::Finite_edges_iterator
            Finite_edges_iterator;

    std::map<Vertex_handle, unsigned int> index;
    // write vertices
    f << "# Shewchuk Triangle .poly file, produced by the CGAL::Mesh_2 package"
      << std::endl
      << "# Neither attributes nor boundary markers are used." << std::endl
      << t.number_of_vertices() << " " << 2 << " "
      << 0 << " " << 0 << std::endl;

    f << std::endl;

    unsigned int vertices_counter = 0;
    for (Finite_vertices_iterator vit = t.finite_vertices_begin();
         vit != t.finite_vertices_end();
         ++vit)
    {
        f << ++vertices_counter << " " << vit->point() << std::endl;
        index[vit] = vertices_counter;
    }

    f << std::endl;
}

void
print(const CTP& cdtp, CTP::Constraint_id cid)
{
    std::cout << "Polyline constraint:" << std::endl;
    for(CTP::Vertex_handle vh : cdtp.vertices_in_constraint(cid)){
        std::cout << vh->point() << std::endl;
    }
}

void
contexts(const CTP& cdtp)
{
    for(auto sc : cdtp.subconstraints()){
        CTP::Vertex_handle vp = sc.first.first, vq = sc.first.second;
        if(cdtp.number_of_enclosing_constraints(vp, vq) == 2){
            std::cout << "subconstraint " << vp->point() << " " << vq->point()
                      << " is on constraints starting at:\n";
            for(const CTP::Context& c : cdtp.contexts(vp,vq)){
                std::cout << (*(c.vertices_begin()))->point() << std::endl;
            }
        }
    }
}

void mark_domains(CTP& ct, CTP::Face_handle start, int index, std::list<CTP::Edge>& border)
{
    if (start->info().nesting_level != -1){  // traversed
        return;
    }
    std::list<CDT::Face_handle> queue;
    queue.push_back(start);
    while (!queue.empty()){
        CTP::Face_handle fh = queue.front();
        queue.pop_front();
        if (fh->info().nesting_level == -1){
            fh->info().nesting_level = index;
            for (int i = 0; i < 3; i++){
                CDT::Edge e(fh, i);
                CDT::Face_handle n = fh->neighbor(i);
                if (n->info().nesting_level == -1){
                    if (ct.is_constrained(e))
                        border.push_back(e);
                    else
                        queue.push_back(n);
                }
            }
        }
    }
}


//explore set of facets connected with non constrained edges,
//and attribute to each such set a nesting level.
//We start from facets incident to the infinite vertex, with a nesting
//level of 0. Then we recursively consider the non-explored facets incident
//to constrained edges bounding the former set and increase the nesting level by 1.
//Facets in the domain are those with an odd nesting level.
void mark_domains(CTP& cdt)
{
    for (CTP::All_faces_iterator it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it){
        it->info().nesting_level = -1;
    }
    std::list<CDT::Edge> border;
    mark_domains(cdt, cdt.infinite_face(), 0, border);
    while (!border.empty()){
        CTP ::Edge e = border.front();
        border.pop_front();
        CTP::Face_handle n = e.first->neighbor(e.second);
        if (n->info().nesting_level == -1){
            mark_domains(cdt, n, e.first->info().nesting_level + 1, border);
        }
    }
}


int main() {
    std::cout << "Step-1: Load raw files" << std::endl;
    std::string arrow_filename = "/home/mxx/Desktop/data-test-roadmodel/181013_030701-11-19-58-738arrowvalidvalid.shp";
    std::string edge_filename = "/home/mxx/Desktop/data-test-roadmodel/181013_030701-11-19-58-738edgevalid.shp";
    std::string lane_filename = "/home/mxx/Desktop/data-test-roadmodel/181013_030701-11-19-58-738lanevalid.shp";
    std::string isolat_filename = "/home/mxx/Desktop/data-test-roadmodel/181013_030701-11-19-58-738isolationvalid.shp";

    std::vector<std::vector<Eigen::Vector3d>> arrows;
    std::vector<std::vector<Eigen::Vector3d>> lanes;
    std::vector<std::vector<Eigen::Vector3d>> edges;
    std::vector<std::vector<Eigen::Vector3d>> isolations;
    utility::shpline_io_shplib::readPolylineShp(arrow_filename, arrows);
    utility::shpline_io_shplib::readPolylineShp(edge_filename, edges);
    utility::shpline_io_shplib::readPolylineShp(lane_filename, lanes);
    utility::shpline_io_shplib::readPolylineShp(isolat_filename, isolations);

    std::cout << "Step-2: Construct TIN" << std::endl;
//    auto pts_edges = convert_point_struct2(edges);
//    auto pts_lanes = convert_point_struct2(lanes);
//    auto pts_arrows = convert_point_struct2(arrows);
////    pts_lanes.add_normal_map();		//添加法向量，默认为（0，0，0）
//    // Create DSM
//    TIN dsm (pts_lanes.points().begin(), pts_lanes.points().end());

    std::cout << "Step-2-1: Construct Constraint Triangulations. " << std::endl;
    CTP ctp;

    auto constraint_edges = convert_point_struct_lines(edges);
    auto constraint_lanes = convert_point_struct_lines(lanes);
    auto constraint_isolations = convert_point_struct_lines(isolations);
    auto constraint_arrows = convert_point_struct_polygons(arrows);

    std::cout << "CTP, vertex number 1: " << ctp.number_of_vertices() << std::endl;
    std::cout << "CTP, face number 1: " <<  ctp.number_of_faces() << std::endl;
    std::vector<CTP::Constraint_id> cons_isolations_ids(constraint_isolations.size());
    for (int i = 0; i < constraint_isolations.size(); ++i) {
        ctp.insert_constraint(constraint_isolations[i].begin(), constraint_isolations[i].end());
    }

    std::vector<CTP::Constraint_id> cons_edges_ids(constraint_edges.size());
    for (int i = 0; i < constraint_edges.size(); ++i) {
        ctp.insert_constraint(constraint_edges[i].begin(), constraint_edges[i].end());
    }
    std::cout << "CTP, vertex number 2: " << ctp.number_of_vertices() << std::endl;
    std::cout << "CTP, face number 2: " <<  ctp.number_of_faces() << std::endl;
    std::vector<CTP::Constraint_id> cons_lanes_ids(constraint_lanes.size());
    for (int i = 0; i < constraint_lanes.size(); ++i) {
        ctp.insert_constraint(constraint_lanes[i].begin(), constraint_lanes[i].end());
    }

    std::cout << "CTP, vertex number 3: " << ctp.number_of_vertices() << std::endl;
    std::cout << "CTP, face number 3: " <<  ctp.number_of_faces() << std::endl;
    std::vector<CTP::Constraint_id> cons_arrows_ids(constraint_lanes.size());
    for (int i = 0; i < constraint_arrows.size(); ++i) {
        ctp.insert_constraint(constraint_arrows[i].vertices_begin(), constraint_arrows[i].vertices_end(), true);
    }
    std::cout << "CTP, vertex number 4: " << ctp.number_of_vertices() << std::endl;
    std::cout << "CTP, face number 4: " <<  ctp.number_of_faces() << std::endl;
//    std::cout << ctp << std::endl;

    //Mark facets that are inside the domain bounded by the polygon
    mark_domains(ctp);

    int idx = 0;
    for (CDT::Vertex_iterator v = ctp.vertices_begin(); v != ctp.vertices_end(); ++v)
    {
        CDT::Vertex_handle vv = v->handle();
        vv->id() = idx;
        idx++;
    }
    std::cout << "idx count: " << idx << std::endl;

    int count = 0;
    int out_count = 0;
    for (CDT::Finite_faces_iterator fit = ctp.finite_faces_begin();
         fit != ctp.finite_faces_end(); ++fit)
    {
        if (fit->info().in_domain())   // odd
        {
            ++count;
            for (int i = 0; i < 3; i++)
            {
                CDT::Vertex_handle vert = fit->vertex(i);
                int x=vert->id();
//                std::cout << "The Id is " << x << std::endl;
                CDT::Edge ed(fit, x);
                ed.second;
//                ed.second = (i+1)%3;
            }
        }
        else {
            ++out_count;
            for (int i = 0; i < 3; i++) {
                CDT::Vertex_handle vert = fit->vertex(i);
                int x = vert->id();
                CDT::Edge ed(fit, x);
                ed.second;
//                ed.second = (i+1)%3;
            }
        }
    }

//    std::cout << "finit face count: " << count << std::endl;
    std::cout << "Step-3-0: graph to mesh. " << std::endl;



    std::cout << "Step-3: Save/Show constructed model" << std::endl;
//    std::ofstream dsm_ofile ("constrait_triangulation.ply", std::ios_base::binary);
//    CGAL::IO::set_binary_mode (dsm_ofile);
//    CGAL::IO::write_PLY (dsm_ofile, all_pts.size(), plylines);
//    dsm_ofile.close();
//    CGAL::IO::write_OBJ("cconstrait_triangulation.bin", ctp);
//    std::ofstream cdt_ofile ("constrait_triangulation.wkt", std::ios_base::binary);
//    cdt_ofile.precision(18);
//    CGAL::IO::write_multi_linestring_WKT(cdt_ofile, plylines);
//    cdt_ofile.close();

    FILE *ply = fopen("my_constraint_delaunay.ply", "w");
    if(ply){
        fprintf(ply, "ply\nformat %s 1.0\n", "ascii");
        fprintf(ply, "element vertex %d\n",ctp.number_of_vertices() );
        fprintf(ply, "property float x\n");
        fprintf(ply, "property float y\n");
        fprintf(ply, "property float z\n");
        fprintf(ply, "element face %d\n",  out_count);  // in hole: count; out_hole: out_count
        fprintf(ply, "property list uint8 int32 vertex_indices\n");
        fprintf(ply, "end_header\n");

        for (CTP::Vertex_iterator v = ctp.vertices_begin(); v != ctp.vertices_end(); ++v)
        {
            CTP::Vertex_handle vv = v->handle();
            double x = vv->point().x();
            double y = vv->point().y();
            double z = vv->point().z();
            fprintf(ply, "%f %f %f\n", x, y, z);
        }
        for (CTP::Finite_faces_iterator fit = ctp.finite_faces_begin();    fit != ctp.finite_faces_end(); ++fit)
        {
            if (fit->info().in_domain())
            {

//                CTP::Vertex_handle vertId0 = fit->vertex(0);
//                CTP::Vertex_handle vertId1 = fit->vertex(1);
//                CTP::Vertex_handle vertId2 = fit->vertex(2);
//                int id0 = vertId0->id();
//                int id1 = vertId1->id();
//                int id2 = vertId2->id();
//                fprintf(ply, "%d %d %d %d\n", 3, id0, id1, id2);
            }
            else{
                CDT::Vertex_handle vertId0 = fit->vertex(0);
                CDT::Vertex_handle vertId1 = fit->vertex(1);
                CDT::Vertex_handle vertId2 = fit->vertex(2);
                int id0 = vertId0->id();
                int id1 = vertId1->id();
                int id2 = vertId2->id();
                fprintf(ply, "%d %d %d %d\n", 3, id0, id1, id2);
            }
        }
    }


    return 1;

}
