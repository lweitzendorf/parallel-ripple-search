(* Random Graph Generation *)
(* Gavin Gray, ETHZ 2021 *)

open Graph

module IGrph = struct
  module IntInt = struct
    type t = int * int
    let compare = Stdlib.compare
    let equal = (=)
    let hash = Hashtbl.hash
  end
  module Int = struct
    type t = int
    let compare = Stdlib.compare
    let hash = Hashtbl.hash
    let equal = (=)
    let default = 0
  end

  include Persistent.Graph.Concrete(IntInt)

  let rec v_table_names = Hashtbl.create 50000
  and v_count = ref 0
  and v_table_ref v =
    try
      Hashtbl.find v_table_names v
    with Not_found ->
      let name = Printf.sprintf "node%d" !v_count in
      incr v_count;
      Hashtbl.add v_table_names v name;
      name

  let vertex_name = v_table_ref (* HACK mutation! *)
  let vertex_attributes v =
    let (x, y) = v in
    [ `Pos (float_of_int x, float_of_int y)
    ; `Label (vertex_name v) ]
  let default_vertex_attributes _ = []
  let edge_attributes _ = [`Dir `None]
  let default_edge_attributes _ = [`Dir `None]
  let get_subgraph _ = None
  let graph_attributes _ = []
end

(* GENERAL FNS *)

let rnd_idx arr = Array.get arr (Array.length arr |> Random.int)
let add_posn (x, y) (x', y') = (x + x', y + y')
let cross_dirs = Array.of_list [ (1, 0); (0, 1); (-1, 0); (0, -1); ]
let corner_dirs = Array.of_list [ (1, 0); (0, -1); ]
let all_dirs = Array.of_list [ (-1, 1)  ; (0, 1)  ; (1, 1)
                             ; (-1, 0)  ; (0, 0)  ; (1, 0)
                             ; (-1, -1) ; (0, -1) ; (1, -1) ]

let write_graph_as_ppm graph ~out ~width ~height =
  let (w, b) = 255, 0 in
  begin
    Printf.fprintf out "P3\n%d %d\n255\n" width height;
    for y = 0 to pred height do
      for x = 0 to pred width do
        if IGrph.mem_vertex graph (x, y) then
          Printf.fprintf out "%d %d %d " w w w
        else Printf.fprintf out "%d %d %d " b b b
      done
    done;
    output_char out '\n';
    flush out
  end

(* TUNNEL ALGO *)

let rec make_tunnel_graph ?min_depth:(md=0) ~xrange ~yrange ~tunnels ~depth () =
  make_tunnel_matrix ~xrange ~yrange ~tunnels ~depth ~min_depth:md ()
  |> matrix_to_graph ~xrange ~yrange

and make_tunnel_matrix ?min_depth:(md=0) ~xrange ~yrange ~tunnels ~depth () =
  let in_range = fun (x, y) ->
    x >= 0 && y >= 0 && x < xrange && y < yrange in
  let mat = Array.make_matrix yrange xrange true in
  let rec o_loop ts pos =
    let dir = rnd_idx cross_dirs in
    let rec i_loop len pos =
      let next_pos = add_posn pos dir in
      if len = 0 || (in_range next_pos |> not) then
        pos
      else
        begin
          let (x', y') = next_pos in
          mat.(y').(x') <- false; (* HACK mutation! *)
          i_loop (len - 1) next_pos
        end
    in
    if ts <> 0 then
      o_loop (ts - 1) (i_loop (Random.int depth + md) pos)
  in
  begin
    (Random.int xrange, Random.int yrange)
    |> o_loop tunnels;
    mat
  end

(* AUTOMATA ALGO *)

and make_automata_graph ?initial_prob:(ip=50) ?rounds:(rnds=4) ~xrange ~yrange =
  (* NOTE walls are represented by TRUE *)
  Array.init yrange (fun _ ->
      Array.init xrange (fun _ ->
          Random.int 100 < ip))
  |> automata_round rnds ~xrange ~yrange
  |> matrix_to_graph ~xrange ~yrange

and automata_round ?additional:(af=fun _ -> true) ts mat ~xrange ~yrange =
  let is_valid = fun x y ->
    x >= 0 && y >= 0 && x < xrange && y < yrange in
  let int_of_bool = fun b -> if b then 1 else 0 in
  let loop = automata_round ~xrange ~yrange in
  if ts = 0 then
    mat
  else Array.init yrange (fun y ->
      Array.init xrange (fun x ->
          Array.map (fun dir ->
              add_posn (x, y) dir
              |> (fun (x, y) ->
                  (is_valid x y && mat.(y).(x))
                  |> int_of_bool)) all_dirs
          |> Array.fold_left (+) 0
          |> (fun v -> v >= 5 && af v)))
       |> loop (ts - 1)

and matrix_to_graph arr ~xrange ~yrange =
  let is_valid =  fun x y ->
    x >= 0 && y >= 0 && x < xrange && y < yrange in
  let rec o_loop y graph =
    let rec i_loop x graph =
      if x = xrange then
        graph
      else (if not arr.(y).(x) then
              Array.fold_left (fun g dir ->
                  add_posn (x, y) dir
                  |> (fun (x', y') ->
                      if is_valid x' y' && not arr.(y').(x') then
                        IGrph.add_edge g (x, y) (x', y')
                      else g)) graph corner_dirs
            else graph) |> i_loop (x + 1)
    in
    if y = yrange then
      graph
    else
      i_loop 0 graph
      |> o_loop (y + 1)
  in o_loop 0 IGrph.empty

(* DELAUNAY TRIANGLE ALGO *)

and make_triangle_graph ~num_v ~xrange ~yrange =
  let module D = Delaunay.Int in
  Array.init num_v (fun _ ->
      Random.int xrange , Random.int yrange)
  |> D.triangulate
  |> (fun t ->
      D.fold (fun v1 v2 g ->
          IGrph.add_edge g v1 v2) t IGrph.empty)

(* POLYGON VORONOI GRAPH *)

(* MAIN *)

let () =
  let open  Graphviz.Neato(IGrph) in
  (* TODO command line parsing for when multiple *)
  let xrange = 1000
  and yrange = 1000
  and num_v  = 200

  and tunnels = 1500
  and depth = 300
  and min_depth = 75
  in

  Random.self_init ();
  let base = Printf.sprintf "mat-%d-%d" xrange yrange in
  let fout = Printf.sprintf "graphs/%s.dot" base |> open_out
  and bout = Printf.sprintf "imgs/%s.ppm" base |> open_out in

  (* ignore num_v;
   * let graph = make_tunnel_graph ~xrange ~yrange ~tunnels ~depth ~min_depth () in *)

  (* ignore num_v;
   * let graph = make_tunnel_matrix ~xrange ~yrange ~tunnels ~depth ~min_depth
   *             |> automata_round 8 ~xrange ~yrange ~additional:(fun v -> v > 7)
   *             |> matrix_to_graph ~xrange ~yrange in *)


  ignore tunnels; ignore depth; ignore min_depth; ignore num_v;
  let graph = make_automata_graph ~initial_prob:52 ~rounds:7 ~xrange ~yrange in

  (* ignore tunnels; ignore depth; ignore min_depth;
   * let graph = make_triangle_graph ~num_v ~xrange ~yrange in *)

  begin
    output_graph fout graph;
    write_graph_as_ppm graph ~out:bout ~width:xrange ~height:yrange;
    close_out fout;
    close_out bout;
  end
