//
// Created by mxx on 9-12-22.
//

//    std::vector<std::vector<Point_3>> plylines;
//    std::vector<Point_3> all_pts;
//    for (CTP::Constraint_id cid : ctp.constraints()) {
//        plylines.push_back(std::vector<Point_3>());
//        plylines.back().reserve(ctp.vertices_in_constraint(cid).size());
//        for (CTP::Vertex_handle vh : ctp.vertices_in_constraint(cid)) {
//            plylines.back().push_back(vh->point());
//            all_pts.push_back(vh->point());
//        }
//    }
//    std::cout << "total vertex size: " << all_pts.size() << std::endl;
//    double spacing = CGAL::compute_average_spacing<Concurrency_tag>(all_pts, 6);
// Simplification algorithm with limit on distance
//    PS::simplify (ctp, PS::Squared_distance_cost(), PS::Stop_above_cost_threshold (16 * spacing * spacing));

//    std::size_t nb_vertices = std::accumulate(
//            plylines.begin(), plylines.end(), std::size_t(0),
//            [](std::size_t size, const std::vector<Point_3>& poly) -> std::size_t
//            {return size + poly.size();});
//
//    std::cout << "accumulated vertex size: " << nb_vertices << std::endl;

//    contexts(ctp);


//    using Mesh = CGAL::Surface_mesh<Point_3>;
//    Mesh dsm_mesh;
//    CGAL::copy_face_graph (ctp, dsm_mesh,
//                           CGAL::parameters::face_to_face_output_iterator
//                                   (boost::make_function_output_iterator
//                                            ([&](const std::pair<CDT_TDS::Face_handle, Mesh::Face_index>& ff)
//                                             {
//                                                 double longest_edge = 0.;
//                                                 bool border = false;
//                                                 for (int i = 0; i < 3; ++ i)
//                                                 {
//                                                     longest_edge = (std::max)(longest_edge, CGAL::squared_distance
//                                                             (ff.first->vertex((i+1)%3)->point(),
//                                                              ff.first->vertex((i+2)%3)->point()));
//
//                                                     CDT_TDS::Face_circulator circ
//                                                             = ctp.incident_faces (ff.first->vertex(i)),
//                                                             start = circ;
//                                                     do
//                                                     {
//                                                         if (ctp.is_infinite (circ))
//                                                         {
//                                                             border = true;
//                                                             break;
//                                                         }
//                                                     }
//                                                     while (++ circ != start);
//
//                                                     if (border)
//                                                         break;
//                                                 }
//
////                                                 // Select if face is too big AND it's not
////                                                 // on the border (to have closed holes)
////                                                 if (!border && longest_edge > limit)
////                                                 {
////                                                     face_selection_map[ff.second] = true;
////                                                     face_selection.push_back (ff.second);
////                                                 }
//                                             }))
//                           (boost::make_function_output_iterator(CGAL::parameters::vertex_to_vertex_map(
//                    [&](const std::pair<CDT_TDS::Vertex_handle , Mesh::Vertex_index>& ff)
//                    {
//                       ff.first->ID;
//                    })))
//                    );
